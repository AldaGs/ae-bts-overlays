/*
	BTSOverlay.cpp — BUILD 2

	KILL-GATE #1: can an adjustment-layer effect read a layer's data via AEGP
	suites acquired from inside the effect, and draw a correct overlay into the
	output world?

	This build:
	  - Walks the scene from the effect: GetEffectLayer -> GetLayerParentComp ->
	    enumerate layers -> read a layer's POSITION stream at the current time.
	  - PreRender picks the layer directly below our adjustment layer, reads its
	    Position (full-res comp px), stashes it in the BTSInfo POD.
	  - SmartRender passes the composite through (all depths) and, at 32-bit
	    float, rasterizes ONE amber crosshair at that position.
	  - The "Dump Diagnostics" button now walks ALL layers on the MAIN thread and
	    writes their Position + our mapped screen px to the CSV (lane 1 of the
	    verification harness).

	The AEGP walk happens in PreRender (render threads, MFR on) — that is exactly
	the thing this build is testing. Drawing is 32f-only for the POC; 8/16-bit
	pass through unchanged (set the project to 32 bpc to see the crosshair).

	Coordinate mapping (2D, no camera yet):
	    buffer_px = comp_fullres_x * downsample - output_origin_x
	    buffer_py = comp_fullres_y * downsample - output_origin_y
	output_origin is per-SmartRender-call (tiles/ROI), so we map at draw time.
*/

#define _CRT_SECURE_NO_WARNINGS

#include "BTSOverlay.h"
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <mutex>

static AEGP_PluginID	S_bts_id = 0L;

#define BTS_DIAG_PATH	"C:/AE_SDK/_build_out/bts_diagnostics.csv"
#define BTS_MAX_LAYERS	1024

/* Default amber, used only for the color-param defaults in ParamsSetup. */
#define BTS_DFLT_R	255
#define BTS_DFLT_G	153
#define BTS_DFLT_B	26

static float BTS_c (A_u_char v) { return (float)v / (float)PF_MAX_CHAN8; }
static float BTS_clamp01 (float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

/* AE label colors, index 0 = No Label. PFAppSuite is a UI-side suite and
   PreRender runs on render threads under MFR, so these are snapshotted on the
   main thread (GlobalSetup at load, refreshed in UpdateParamsUI whenever a
   param changes) and PreRender only ever copies the table. The cost of that
   is a Preferences > Labels edit not showing until you next touch a control. */
static float		S_labelCol[BTS_LABEL_COUNT + 1][3];
static A_Boolean	S_labelColValid = FALSE;

static void
BTS_FetchLabelColors (PF_InData *in_data)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);

	for (A_long n = 0; n <= BTS_LABEL_COUNT; ++n) {
		PF_App_Color c;  AEFX_CLR_STRUCT(c);
		if (suites.AppSuite6()->PF_AppGetColor(
				(PF_App_ColorType)(PF_App_Color_LABEL_0 + n), &c) == PF_Err_NONE) {
			// PF_App_Color is a UI color: plain 16-bit shorts, full 0..65535
			// range. It is NOT an AE pixel channel, whose white is
			// PF_MAX_CHAN16 == 32768. Scaling by the pixel constant made every
			// label color come out 2x too bright - which reads as washed-out
			// super-white at 16/32 bpc and, because the 8-bit store truncates
			// rather than clamps, as wrapped-around hues at 8 bpc.
			S_labelCol[n][0] = BTS_clamp01((float)c.red   / 65535.0f);
			S_labelCol[n][1] = BTS_clamp01((float)c.green / 65535.0f);
			S_labelCol[n][2] = BTS_clamp01((float)c.blue  / 65535.0f);
		}
	}
	S_labelColValid = TRUE;
}

/* Human-readable kind names, indexed by BTS_LT_* (diagnostics CSV). */
static const char *BTS_kindName[BTS_LT_COUNT] = {
	"camera", "light", "null", "adjustment", "text", "shape", "precomp", "solid", "footage",
	"3dmodel", "3dprim"
};

/* Default per-layer-KIND colors (indexed by BTS_LT_*). Chosen to read apart at
   a glance over footage: warm for the invisible rigging objects (camera/light/
   null), cool for the visible content types. */
static const A_u_char BTS_kindDflt[BTS_LT_COUNT][3] = {
	{ 240,  90,  90 },	// CAMERA   red
	{ 255, 220,  90 },	// LIGHT    yellow
	{ 255, 153,  26 },	// NULL     amber (the original default)
	{ 190, 130, 255 },	// ADJUST   violet
	{  90, 200, 255 },	// TEXT     cyan
	{ 120, 255, 150 },	// SHAPE    green
	{ 255, 130, 200 },	// PRECOMP  pink
	{ 170, 180, 200 },	// SOLID    slate
	{ 255, 255, 255 },	// FOOTAGE  white
	{ 120, 255, 235 },	// MODEL    teal
	{  90, 210, 190 },	// PRIMITIVE  darker teal - a sibling of MODEL, not the same thing
};

/* =========================================================================
   Boilerplate
   ========================================================================= */

static PF_Err
About (PF_InData *in_data, PF_OutData *out_data, PF_ParamDef *params[], PF_LayerDef *output)
{
	AEGP_SuiteHandler suites(in_data->pica_basicP);
	suites.ANSICallbacksSuite1()->sprintf(
		out_data->return_msg,
		"%s v%d.%d\r%s",
		STR(StrID_Name), MAJOR_VERSION, MINOR_VERSION, STR(StrID_Description));
	return PF_Err_NONE;
}

static PF_Err
GlobalSetup (PF_InData *in_data, PF_OutData *out_data, PF_ParamDef *params[], PF_LayerDef *output)
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);

	out_data->my_version = PF_VERSION(MAJOR_VERSION, MINOR_VERSION,
									  BUG_VERSION, STAGE_VERSION, BUILD_VERSION);

	// NON_PARAM_VARY: our output is generated from OTHER LAYERS' data (positions,
	// keyframes, camera) read via AEGP — state AE's dependency tracking knows
	// nothing about. Without it AE serves cached frames and the overlay appears
	// frozen/stale even though PreRender's geometry is correct.
	out_data->out_flags  =	PF_OutFlag_DEEP_COLOR_AWARE |
							PF_OutFlag_USE_OUTPUT_EXTENT |
							PF_OutFlag_NON_PARAM_VARY |
	// SEND_UPDATE_PARAMS_UI: lets us grey out controls that cannot currently
	// affect a pixel (see UpdateParamsUI). Mirrored in the PiPL out-flags.
							PF_OutFlag_SEND_UPDATE_PARAMS_UI;

	out_data->out_flags2 =	PF_OutFlag2_SUPPORTS_SMART_RENDER |
							PF_OutFlag2_FLOAT_COLOR_AWARE |
							PF_OutFlag2_SUPPORTS_THREADED_RENDERING;

	ERR(suites.UtilitySuite3()->AEGP_RegisterWithAEGP(NULL, STR(StrID_Name), &S_bts_id));
	BTS_FetchLabelColors(in_data);		// main thread; refreshed in UpdateParamsUI
	return err;
}

static PF_Err
ParamsSetup (PF_InData *in_data, PF_OutData *out_data, PF_ParamDef *params[], PF_LayerDef *output)
{
	PF_Err		err = PF_Err_NONE;
	PF_ParamDef	def;

	// View + Dump Diagnostics are dev instruments; BTS_DEV_UI hides them for
	// ship builds. PF_ADD_POPUP does not touch def.ui_flags, so setting it here
	// survives the macro.
	AEFX_CLR_STRUCT(def);
	def.ui_flags = BTS_DEV_PUI;
	PF_ADD_POPUP(	STR(StrID_View_Param_Name), 2, BTS_VIEW_FINAL,
					STR(StrID_View_Choices), VIEW_DISK_ID);

	// Transform warning. A read-only status line, not a control: permanently
	// DISABLED so nobody can click it, and INVISIBLE until UpdateParamsUI finds
	// our host layer transformed. It is a checkbox purely because a checkbox is
	// the one param type that carries a second, long text string (the comment
	// beside the box) - the name field caps at 31 characters, too short for an
	// instruction. Nothing ever reads its value.
	AEFX_CLR_STRUCT(def);
	def.ui_flags = PF_PUI_DISABLED | PF_PUI_INVISIBLE;
	PF_ADD_CHECKBOX(STR(StrID_Warn_Param_Name), STR(StrID_Warn_Comment),
					FALSE, 0, WARN_DISK_ID);

	// The old three-way Output popup split into two checkboxes, because it was
	// really two independent questions: do we draw overlays at all (the A/B
	// compare you reach for constantly, now one click), and do we keep the
	// composite underneath or hand back overlays on transparency.
	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_ShowOverlays_Param_Name), TRUE, 0, SHOW_OVERLAYS_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_OverlaysOnly_Param_Name), FALSE, 0, OVERLAYS_ONLY_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX(STR(StrID_Opacity_Param_Name), 0, 100, 0, 100, 100,
						 PF_Precision_TENTHS, 0, 0, OPACITY_DISK_ID);

	// Where every gizmo takes its color from. Defaults to By Label: a comp is
	// usually already organized by label, so the overlays read correctly with
	// no setup. Whichever source is picked greys out the pickers it overrides.
	AEFX_CLR_STRUCT(def);
	PF_ADD_POPUP(STR(StrID_ColorSource_Param_Name), 3, BTS_COLSRC_LABEL,
				 STR(StrID_ColorSource_Choices), COLOR_SOURCE_DISK_ID);

	// --- Overlay Timing group -----------------------------------------------
	AEFX_CLR_STRUCT(def);
	PF_ADD_TOPIC(STR(StrID_Timing_Topic), TIMING_TOPIC_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_POPUP(STR(StrID_Visibility_Param_Name), 2, BTS_VIS_WHOLE,
				 STR(StrID_Visibility_Choices), VISIBILITY_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX(STR(StrID_WinStart_Param_Name), 0, 60, 0, 30, 5,
						 PF_Precision_HUNDREDTHS, 0, 0, WIN_START_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX(STR(StrID_WinLen_Param_Name), 0, 60, 0, 30, 5,
						 PF_Precision_HUNDREDTHS, 0, 0, WIN_LEN_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_END_TOPIC(TIMING_TOPIC_END_DISK_ID);

	// --- Layer Markers group ------------------------------------------------
	AEFX_CLR_STRUCT(def);
	PF_ADD_TOPIC(STR(StrID_Markers_Topic), MARK_TOPIC_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_ShowMarkers_Param_Name), TRUE, 0, SHOW_MARKERS_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_POPUP(STR(StrID_MarkerStyle_Param_Name), 3, BTS_MSTYLE_ANCHOR,
				 STR(StrID_MarkerStyle_Choices), MARKER_STYLE_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_COLOR(STR(StrID_MarkerColor_Param_Name), BTS_DFLT_R, BTS_DFLT_G, BTS_DFLT_B, MARKER_COLOR_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX(STR(StrID_MarkerSize_Param_Name), 1, 100, 1, 40, 12,
						 PF_Precision_INTEGER, 0, 0, MARKER_SIZE_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_ShowAxes_Param_Name), TRUE, 0, SHOW_AXES_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX(STR(StrID_AxisLen_Param_Name), 10, 2000, 10, 500, 100,
						 PF_Precision_INTEGER, 0, 0, AXIS_LEN_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX(STR(StrID_AxisWidth_Param_Name), 1, 20, 1, 8, 2,
						 PF_Precision_INTEGER, 0, 0, AXIS_WIDTH_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_POPUP(STR(StrID_AxisHead_Param_Name), 2, BTS_AHEAD_BARBS,
				 STR(StrID_AxisHead_Choices), AXIS_HEAD_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_END_TOPIC(MARK_TOPIC_END_DISK_ID);

	// --- Motion Paths group -------------------------------------------------
	AEFX_CLR_STRUCT(def);
	PF_ADD_TOPIC(STR(StrID_Paths_Topic), PATH_TOPIC_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_ShowPaths_Param_Name), TRUE, 0, SHOW_PATHS_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_COLOR(STR(StrID_PathColor_Param_Name), BTS_DFLT_R, BTS_DFLT_G, BTS_DFLT_B, PATH_COLOR_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX(STR(StrID_PathThick_Param_Name), 1, 20, 1, 8, 1,
						 PF_Precision_INTEGER, 0, 0, PATH_THICK_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_ShowKfDots_Param_Name), TRUE, 0, SHOW_KFDOTS_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_ShowTangents_Param_Name), TRUE, 0, SHOW_TANGENTS_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_ShowFrameDots_Param_Name), TRUE, 0, SHOW_FRAMEDOTS_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_END_TOPIC(PATH_TOPIC_END_DISK_ID);

	// --- Bounding Boxes group -----------------------------------------------
	AEFX_CLR_STRUCT(def);
	PF_ADD_TOPIC(STR(StrID_Boxes_Topic), BOX_TOPIC_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_ShowBoxes_Param_Name), TRUE, 0, SHOW_BOXES_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_COLOR(STR(StrID_BoxColor_Param_Name), BTS_DFLT_R, BTS_DFLT_G, BTS_DFLT_B, BOX_COLOR_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX(STR(StrID_BoxThick_Param_Name), 1, 20, 1, 8, 1,
						 PF_Precision_INTEGER, 0, 0, BOX_THICK_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_ShowHandles_Param_Name), TRUE, 0, SHOW_HANDLES_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_COLOR(STR(StrID_HandleColor_Param_Name), 51, 204, 255, HANDLE_COLOR_DISK_ID);	// AE-ish cyan

	AEFX_CLR_STRUCT(def);
	PF_ADD_FLOAT_SLIDERX(STR(StrID_HandleSize_Param_Name), 2, 40, 2, 20, 8,
						 PF_Precision_INTEGER, 0, 0, HANDLE_SIZE_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_END_TOPIC(BOX_TOPIC_END_DISK_ID);

	// --- Layer Types group ---------------------------------------------------
	// Per-KIND show filter + color. "Color By Type" swaps the global marker/path/
	// box/handle colors for the per-kind color of each layer (3D axis arrows keep
	// their fixed X/Y/Z red/green/blue). Guide/Locked are ATTRIBUTE filters: they
	// cut across kinds, so they are include-checkboxes rather than colors.
	AEFX_CLR_STRUCT(def);
	PF_ADD_TOPIC(STR(StrID_Types_Topic), TYPE_TOPIC_DISK_ID);


#define BTS_ADD_KIND(SHOW_STR, COL_STR, DFLT_ON, KIND, SHOW_ID, COL_ID)				AEFX_CLR_STRUCT(def);															PF_ADD_CHECKBOXX(STR(SHOW_STR), (DFLT_ON), 0, SHOW_ID);							AEFX_CLR_STRUCT(def);															PF_ADD_COLOR(STR(COL_STR), BTS_kindDflt[KIND][0], BTS_kindDflt[KIND][1],					 BTS_kindDflt[KIND][2], COL_ID)

	BTS_ADD_KIND(StrID_Cameras_Param_Name,  StrID_CamerasColor_Param_Name,  TRUE,  BTS_LT_CAMERA,  SHOW_CAMERAS_DISK_ID,  COL_CAMERAS_DISK_ID);
	BTS_ADD_KIND(StrID_Lights_Param_Name,   StrID_LightsColor_Param_Name,   TRUE,  BTS_LT_LIGHT,   SHOW_LIGHTS_DISK_ID,   COL_LIGHTS_DISK_ID);
	BTS_ADD_KIND(StrID_Nulls_Param_Name,    StrID_NullsColor_Param_Name,    TRUE,  BTS_LT_NULL,    SHOW_NULLS_DISK_ID,    COL_NULLS_DISK_ID);
	BTS_ADD_KIND(StrID_Adjust_Param_Name,   StrID_AdjustColor_Param_Name,   TRUE,  BTS_LT_ADJUST,  SHOW_ADJUST_DISK_ID,   COL_ADJUST_DISK_ID);
	BTS_ADD_KIND(StrID_Text_Param_Name,     StrID_TextColor_Param_Name,     TRUE,  BTS_LT_TEXT,    SHOW_TEXT_DISK_ID,     COL_TEXT_DISK_ID);
	BTS_ADD_KIND(StrID_Shapes_Param_Name,   StrID_ShapesColor_Param_Name,   TRUE,  BTS_LT_SHAPE,   SHOW_SHAPES_DISK_ID,   COL_SHAPES_DISK_ID);
	BTS_ADD_KIND(StrID_Precomps_Param_Name, StrID_PrecompsColor_Param_Name, TRUE,  BTS_LT_PRECOMP, SHOW_PRECOMPS_DISK_ID, COL_PRECOMPS_DISK_ID);
	BTS_ADD_KIND(StrID_Solids_Param_Name,   StrID_SolidsColor_Param_Name,   TRUE,  BTS_LT_SOLID,   SHOW_SOLIDS_DISK_ID,   COL_SOLIDS_DISK_ID);
	BTS_ADD_KIND(StrID_Footage_Param_Name,  StrID_FootageColor_Param_Name,  TRUE,  BTS_LT_FOOTAGE, SHOW_FOOTAGE_DISK_ID,  COL_FOOTAGE_DISK_ID);
	BTS_ADD_KIND(StrID_Models_Param_Name,   StrID_ModelsColor_Param_Name,   TRUE,  BTS_LT_MODEL,   SHOW_MODELS_DISK_ID,   COL_MODELS_DISK_ID);
	BTS_ADD_KIND(StrID_Prims_Param_Name,    StrID_PrimsColor_Param_Name,    TRUE,  BTS_LT_PRIMITIVE, SHOW_PRIMS_DISK_ID,  COL_PRIMS_DISK_ID);
#undef BTS_ADD_KIND

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_Guides_Param_Name), TRUE, 0, SHOW_GUIDES_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_ADD_CHECKBOXX(STR(StrID_Locked_Param_Name), TRUE, 0, SHOW_LOCKED_DISK_ID);

	AEFX_CLR_STRUCT(def);
	PF_END_TOPIC(TYPE_TOPIC_END_DISK_ID);

	// PF_ADD_BUTTON clears def itself and takes both flag sets as arguments,
	// so they have to go through the macro to survive.
	AEFX_CLR_STRUCT(def);
	PF_ADD_BUTTON(	STR(StrID_Dump_Param_Name), STR(StrID_Dump_Param_Name),
					BTS_DEV_PUI,
					PF_ParamFlag_SUPERVISE | PF_ParamFlag_CANNOT_TIME_VARY,
					DUMP_DISK_ID);

	out_data->num_params = BTS_NUM_PARAMS;
	return err;
}

/* =========================================================================
   AEGP scene reading (shared by PreRender and the Dump button)
   ========================================================================= */

// Copy an AEGP UTF16 name handle into an ASCII buffer (non-ASCII -> '?'), then
// free the handle. Safe with a NULL handle.
static void
BTS_ReadName (AEGP_SuiteHandler &suites, AEGP_MemHandle nameH, char *out, A_long n)
{
	out[0] = '\0';
	if (!nameH) return;
	A_UTF16Char *u = NULL;
	suites.MemorySuite1()->AEGP_LockMemHandle(nameH, reinterpret_cast<void**>(&u));
	if (u) {
		A_long i = 0;
		for (; i < n - 1 && u[i] != 0; ++i)
			out[i] = (char)(u[i] < 128 ? u[i] : '?');
		out[i] = '\0';
	}
	suites.MemorySuite1()->AEGP_UnlockMemHandle(nameH);
	suites.MemorySuite1()->AEGP_FreeMemHandle(nameH);
}

// Resolve the comp our effect lives in, plus our own layer index and the layer
// count. Any NULL out-pointer is skipped.
static PF_Err
BTS_GetContext (PF_InData *in_data, AEGP_CompH *compP, A_long *myIdxP, A_long *numP)
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_LayerH			meLayer = NULL;
	AEGP_CompH			compH   = NULL;

	ERR(suites.PFInterfaceSuite1()->AEGP_GetEffectLayer(in_data->effect_ref, &meLayer));
	if (!err && meLayer) {
		ERR(suites.LayerSuite9()->AEGP_GetLayerParentComp(meLayer, &compH));
		if (!err && compP)   *compP   = compH;
		if (!err && myIdxP)  ERR(suites.LayerSuite9()->AEGP_GetLayerIndex(meLayer, myIdxP));
		if (!err && numP && compH) ERR(suites.LayerSuite9()->AEGP_GetCompNumLayers(compH, numP));
	}
	return err;
}

/* =========================================================================
   Transform guard.

   Every gizmo is computed in COMP space and written straight into our output
   buffer, on the assumption that our own layer maps 1:1 onto the comp. That
   holds for a freshly created, comp-sized adjustment layer and stops holding
   the moment anyone nudges it: AE applies the LAYER transform AFTER the effect
   renders, so a moved / scaled / rotated / parented host layer drags every
   overlay off the thing it annotates - while the composite underneath, which
   the same transform also moves, still looks plausible. That silent mismatch
   is why this is a hard stop with a visible warning rather than a best-effort
   correction: there is no correction to make from inside the effect.
   ========================================================================= */

// One transform stream evaluated at `t`. Returns FALSE (outputs untouched) when
// the stream cannot be read - callers pre-seed the outputs with the DEFAULT, so
// an unreadable stream degrades to "fine" and never raises a false alarm. That
// matters for the 3D-only streams, which simply do not exist on a 2D layer.
static A_Boolean
BTS_ReadXformStream (AEGP_SuiteHandler &suites, AEGP_LayerH layerH,
                     AEGP_LayerStream which, const A_Time *t,
                     PF_FpLong *x, PF_FpLong *y, PF_FpLong *z)
{
	AEGP_StreamRefH streamH = NULL;
	if (suites.StreamSuite6()->AEGP_GetNewLayerStream(S_bts_id, layerH, which, &streamH) != PF_Err_NONE || !streamH)
		return FALSE;

	AEGP_StreamValue2 sv;  AEFX_CLR_STRUCT(sv);
	A_Boolean ok = (suites.StreamSuite6()->AEGP_GetNewStreamValue(
						S_bts_id, streamH, AEGP_LTimeMode_CompTime, t, FALSE, &sv) == PF_Err_NONE);
	if (ok) {
		if (x) *x = sv.val.three_d.x;
		if (y) *y = sv.val.three_d.y;
		if (z) *z = sv.val.three_d.z;
		suites.StreamSuite6()->AEGP_DisposeStreamValue(&sv);
	}
	suites.StreamSuite6()->AEGP_DisposeStream(streamH);
	return ok;
}

static A_Boolean BTS_Near (PF_FpLong a, PF_FpLong b, PF_FpLong tol)
{ PF_FpLong d = a - b; return (d < 0 ? -d : d) <= tol; }

/* TRUE when our own layer is the pass-through the overlay math assumes:
   unparented, comp-sized, and sitting at the identity transform. Evaluated at
   the CURRENT time, so an ANIMATED transform is caught on exactly the frames
   where it is actually off - which is the honest answer per frame.

   Anything unreadable counts as fine. A guard that fires spuriously would be
   worse than one that occasionally misses, because it blanks the render. */
static A_Boolean
BTS_XformIsDefault (PF_InData *in_data)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_LayerH			meLayer = NULL;
	AEGP_CompH			compH   = NULL;

	if (suites.PFInterfaceSuite1()->AEGP_GetEffectLayer(in_data->effect_ref, &meLayer) != PF_Err_NONE || !meLayer)
		return TRUE;

	// Parenting alone is disqualifying: the parent's transform reaches us the
	// same way our own does, and it is the case people hit by accident.
	AEGP_LayerH parentH = NULL;
	if (suites.LayerSuite9()->AEGP_GetLayerParent(meLayer, &parentH) == PF_Err_NONE && parentH)
		return FALSE;

	if (suites.LayerSuite9()->AEGP_GetLayerParentComp(meLayer, &compH) != PF_Err_NONE || !compH)
		return TRUE;

	// Comp size, and our source's size. A comp-sized adjustment layer defaults
	// to Position = comp centre and Anchor Point = its own centre; a layer that
	// is not comp-sized cannot cover the frame no matter where it sits.
	A_long compW = 0, compH_px = 0, layW = 0, layH = 0;
	AEGP_ItemH compItemH = NULL, srcItemH = NULL;
	if (suites.CompSuite12()->AEGP_GetItemFromComp(compH, &compItemH) != PF_Err_NONE || !compItemH)
		return TRUE;
	if (suites.ItemSuite9()->AEGP_GetItemDimensions(compItemH, &compW, &compH_px) != PF_Err_NONE)
		return TRUE;
	if (suites.LayerSuite9()->AEGP_GetLayerSourceItem(meLayer, &srcItemH) == PF_Err_NONE && srcItemH)
		suites.ItemSuite9()->AEGP_GetItemDimensions(srcItemH, &layW, &layH);
	if (layW > 0 && layH > 0 && (layW != compW || layH != compH_px))
		return FALSE;

	A_Time t;  t.value = in_data->current_time;  t.scale = in_data->time_scale;

	// Each stream is pre-seeded with its default, so a failed read reads clean.
	PF_FpLong px = compW / 2.0, py = compH_px / 2.0, pz = 0.0;
	PF_FpLong ax = (layW ? layW : compW) / 2.0, ay = (layH ? layH : compH_px) / 2.0, az = 0.0;
	PF_FpLong sx = 100.0, sy = 100.0, sz = 100.0;
	PF_FpLong rz = 0.0, rx = 0.0, ry = 0.0, ox = 0.0, oy = 0.0, oz = 0.0;

	BTS_ReadXformStream(suites, meLayer, AEGP_LayerStream_POSITION,    &t, &px, &py, &pz);
	BTS_ReadXformStream(suites, meLayer, AEGP_LayerStream_ANCHORPOINT, &t, &ax, &ay, &az);
	BTS_ReadXformStream(suites, meLayer, AEGP_LayerStream_SCALE,       &t, &sx, &sy, &sz);
	BTS_ReadXformStream(suites, meLayer, AEGP_LayerStream_ROTATE_Z,    &t, &rz, NULL, NULL);
	BTS_ReadXformStream(suites, meLayer, AEGP_LayerStream_ROTATE_X,    &t, &rx, NULL, NULL);
	BTS_ReadXformStream(suites, meLayer, AEGP_LayerStream_ROTATE_Y,    &t, &ry, NULL, NULL);
	BTS_ReadXformStream(suites, meLayer, AEGP_LayerStream_ORIENTATION, &t, &ox, &oy, &oz);

	// Sub-pixel slop only. Half a pixel of drift is already visible against a
	// 1px-wide motion path, which is the whole point of the overlay.
	const PF_FpLong kPos = 0.01, kSca = 0.01, kRot = 0.001;
	if (!BTS_Near(px, compW / 2.0, kPos) || !BTS_Near(py, compH_px / 2.0, kPos) || !BTS_Near(pz, 0.0, kPos))
		return FALSE;
	if (!BTS_Near(ax, (layW ? layW : compW) / 2.0, kPos) ||
		!BTS_Near(ay, (layH ? layH : compH_px) / 2.0, kPos) || !BTS_Near(az, 0.0, kPos))
		return FALSE;
	if (!BTS_Near(sx, 100.0, kSca) || !BTS_Near(sy, 100.0, kSca) || !BTS_Near(sz, 100.0, kSca))
		return FALSE;
	if (!BTS_Near(rz, 0.0, kRot) || !BTS_Near(rx, 0.0, kRot) || !BTS_Near(ry, 0.0, kRot))
		return FALSE;
	if (!BTS_Near(ox, 0.0, kRot) || !BTS_Near(oy, 0.0, kRot) || !BTS_Near(oz, 0.0, kRot))
		return FALSE;

	return TRUE;
}

/* =========================================================================
   3D camera projection. World points from a 3D layer are projected to comp
   screen px via the active camera (or AE's default camera if none). 2D layers
   pass through unchanged (their world already IS comp px). Uses the same
   row-vector matrix convention proven by the 2D bounding boxes.
   ========================================================================= */

typedef struct {
	A_Boolean	isDefault;
	PF_FpLong	cx, cy;			// comp center, full-res px
	PF_FpLong	zoom;			// focal length in px
	PF_FpLong	dist;			// default-camera distance (== zoom when default)
	PF_FpLong	tx, ty, tz;		// real camera position (world)
	PF_FpLong	Linv[3][3];		// world->camera linear inverse (real camera)
} BTSCam;

// Standard 3x3 inverse (adjugate / determinant). FALSE if singular.
static A_Boolean
BTS_Inv3 (const PF_FpLong m[3][3], PF_FpLong o[3][3])
{
	PF_FpLong d = m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
	            - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
	            + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
	if (d < 1e-12 && d > -1e-12) return FALSE;
	PF_FpLong id = 1.0 / d;
	o[0][0] = (m[1][1]*m[2][2]-m[1][2]*m[2][1])*id;
	o[0][1] = (m[0][2]*m[2][1]-m[0][1]*m[2][2])*id;
	o[0][2] = (m[0][1]*m[1][2]-m[0][2]*m[1][1])*id;
	o[1][0] = (m[1][2]*m[2][0]-m[1][0]*m[2][2])*id;
	o[1][1] = (m[0][0]*m[2][2]-m[0][2]*m[2][0])*id;
	o[1][2] = (m[0][2]*m[1][0]-m[0][0]*m[1][2])*id;
	o[2][0] = (m[1][0]*m[2][1]-m[1][1]*m[2][0])*id;
	o[2][1] = (m[0][1]*m[2][0]-m[0][0]*m[2][1])*id;
	o[2][2] = (m[0][0]*m[1][1]-m[0][1]*m[1][0])*id;
	return TRUE;
}

// Project a world point to comp screen px. is3D FALSE -> pass through.
static void
BTS_ProjectPt (const BTSCam *c, A_Boolean is3D, PF_FpLong wx, PF_FpLong wy, PF_FpLong wz,
               PF_FpLong *sx, PF_FpLong *sy)
{
	if (!is3D) { *sx = wx; *sy = wy; return; }
	PF_FpLong px, py, pz;
	if (c->isDefault) {
		px = wx - c->cx;  py = wy - c->cy;  pz = wz + c->dist;
	} else {
		PF_FpLong dx = wx - c->tx, dy = wy - c->ty, dz = wz - c->tz;
		px = dx*c->Linv[0][0] + dy*c->Linv[1][0] + dz*c->Linv[2][0];
		py = dx*c->Linv[0][1] + dy*c->Linv[1][1] + dz*c->Linv[2][1];
		pz = dx*c->Linv[0][2] + dy*c->Linv[1][2] + dz*c->Linv[2][2];
	}
	if (pz < 1e-3 && pz > -1e-3) pz = (pz < 0) ? -1e-3 : 1e-3;	// avoid blowup
	*sx = c->cx + px / pz * c->zoom;
	*sy = c->cy + py / pz * c->zoom;
}

// Build the projection context: find the topmost enabled camera (zoom + world->
// camera inverse), else fall back to AE's default camera.
static void
BTS_BuildCamera (PF_InData *in_data, AEGP_CompH compH, const A_Time *tp, BTSCam *cam)
{
	AEGP_SuiteHandler suites(in_data->pica_basicP);
	AEFX_CLR_STRUCT(*cam);
	cam->isDefault = TRUE;

	AEGP_ItemH compItem = NULL;  A_long cw = 0, ch = 0;
	suites.CompSuite12()->AEGP_GetItemFromComp(compH, &compItem);
	if (compItem) suites.ItemSuite9()->AEGP_GetItemDimensions(compItem, &cw, &ch);
	cam->cx = cw * 0.5;  cam->cy = ch * 0.5;

	A_Time t = *tp;

	AEGP_LayerH camL = NULL;
	A_long num = 0;
	suites.LayerSuite9()->AEGP_GetCompNumLayers(compH, &num);
	for (A_long i = 0; i < num; ++i) {
		AEGP_LayerH L = NULL;
		if (suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, i, &L) || !L) continue;
		AEGP_ObjectType ot = AEGP_ObjectType_NONE;
		suites.LayerSuite9()->AEGP_GetLayerObjectType(L, &ot);
		if (ot == AEGP_ObjectType_CAMERA) {
			AEGP_LayerFlags fl = 0;
			suites.LayerSuite9()->AEGP_GetLayerFlags(L, &fl);
			if (fl & AEGP_LayerFlag_VIDEO_ACTIVE) { camL = L; break; }	// topmost enabled
		}
	}

	if (camL) {
		AEGP_StreamRefH zs = NULL;
		if (!suites.StreamSuite6()->AEGP_GetNewLayerStream(S_bts_id, camL, AEGP_LayerStream_ZOOM, &zs) && zs) {
			AEGP_StreamValue2 sv;  AEFX_CLR_STRUCT(sv);
			if (!suites.StreamSuite6()->AEGP_GetNewStreamValue(S_bts_id, zs, AEGP_LTimeMode_CompTime, &t, FALSE, &sv)) {
				cam->zoom = sv.val.one_d;
				suites.StreamSuite6()->AEGP_DisposeStreamValue(&sv);
			}
			suites.StreamSuite6()->AEGP_DisposeStream(zs);
		}
		A_Matrix4 M;
		if (!suites.LayerSuite9()->AEGP_GetLayerToWorldXform(camL, &t, &M) && cam->zoom > 0) {
			PF_FpLong L3[3][3];
			for (A_long r = 0; r < 3; ++r) for (A_long c = 0; c < 3; ++c) L3[r][c] = M.mat[r][c];
			if (BTS_Inv3(L3, cam->Linv)) {
				cam->tx = M.mat[3][0];  cam->ty = M.mat[3][1];  cam->tz = M.mat[3][2];
				cam->isDefault = FALSE;
			}
		}
	}

	if (cam->isDefault) {
		A_FpLong dist = 0;
		suites.CameraSuite2()->AEGP_GetDefaultCameraDistanceToImagePlane(compH, &dist);
		cam->dist = dist;  cam->zoom = dist;
	}
}

// Read one layer's Position (and optionally name + 3D flag) at the current time.
// x/y/z are FULL-res comp pixels. name may be NULL.
/* -------------------------------------------------------------------------
   Parenting: getting Position into comp space

   A layer's Position - and its Position KEYFRAME values, and its spatial
   tangents - are expressed in its PARENT's coordinate space. Only an
   unparented layer's Position is already comp space.

   Bounding boxes and the 3D axis arrows never had this bug because both go
   through AEGP_GetLayerToWorldXform, which walks the entire parent chain. The
   marker, the motion path and the keyframe dots read the raw Position stream,
   so the moment a layer was parented they drew in the parent's space while the
   box stayed put - the exact symptom.

   The fix is one matrix: the PARENT's layer-to-world transform, which maps
   parent space to comp space. Unparented layers have no parent handle, callers
   skip the multiply entirely, and their behavior is bit-for-bit unchanged.
   ------------------------------------------------------------------------- */

// The layer this one is parented to, or NULL. Resolve once per layer, not per
// sample - the handle is stable across time even though the transform is not.
static AEGP_LayerH
BTS_GetParent (AEGP_SuiteHandler &suites, AEGP_LayerH layerH)
{
	AEGP_LayerH parentH = NULL;
	if (!layerH) return NULL;
	if (suites.LayerSuite9()->AEGP_GetLayerParent(layerH, &parentH) != PF_Err_NONE)
		return NULL;
	return parentH;
}

// Parent-space -> comp-space matrix at one time. FALSE means "no parent, no
// transform needed". The parent may itself be animated and parented, which is
// why this is evaluated per sample time rather than once.
static A_Boolean
BTS_ParentXform (AEGP_SuiteHandler &suites, AEGP_LayerH parentH,
                 const A_Time *t, A_Matrix4 *M)
{
	if (!parentH) return FALSE;
	return suites.LayerSuite9()->AEGP_GetLayerToWorldXform(parentH, t, M) == PF_Err_NONE;
}

// Map a point through a 4x4, row-vector convention (same as BTS_ReadBox).
static void
BTS_XformPt (const A_Matrix4 *M, PF_FpLong *x, PF_FpLong *y, PF_FpLong *z)
{
	PF_FpLong px = *x, py = *y, pz = *z;
	*x = px*M->mat[0][0] + py*M->mat[1][0] + pz*M->mat[2][0] + M->mat[3][0];
	*y = px*M->mat[0][1] + py*M->mat[1][1] + pz*M->mat[2][1] + M->mat[3][1];
	*z = px*M->mat[0][2] + py*M->mat[1][2] + pz*M->mat[2][2] + M->mat[3][2];
}

static PF_Err
BTS_ReadLayer (PF_InData *in_data, AEGP_CompH compH, A_long idx,
               PF_FpLong *xP, PF_FpLong *yP, PF_FpLong *zP,
               A_Boolean *is3dP, char *name, A_long nameLen)
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_LayerH			layerH = NULL;
	AEGP_StreamRefH		streamH = NULL;

	ERR(suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, idx, &layerH));
	if (err || !layerH) return err;

	if (is3dP) {
		AEGP_LayerFlags fl = 0;
		ERR(suites.LayerSuite9()->AEGP_GetLayerFlags(layerH, &fl));
		*is3dP = (fl & AEGP_LayerFlag_LAYER_IS_3D) ? TRUE : FALSE;
	}
	if (name) {
		AEGP_MemHandle nameH = NULL;
		ERR(suites.LayerSuite9()->AEGP_GetLayerName(S_bts_id, layerH, &nameH, NULL));
		BTS_ReadName(suites, nameH, name, nameLen);
	}

	ERR(suites.StreamSuite6()->AEGP_GetNewLayerStream(
			S_bts_id, layerH, AEGP_LayerStream_POSITION, &streamH));
	if (!err && streamH) {
		A_Time t;
		t.value = in_data->current_time;
		t.scale = in_data->time_scale;
		AEGP_StreamValue2 sv;
		AEFX_CLR_STRUCT(sv);
		ERR(suites.StreamSuite6()->AEGP_GetNewStreamValue(
				S_bts_id, streamH, AEGP_LTimeMode_CompTime, &t, FALSE, &sv));
		if (!err) {
			PF_FpLong px = sv.val.three_d.x, py = sv.val.three_d.y, pz = sv.val.three_d.z;
			suites.StreamSuite6()->AEGP_DisposeStreamValue(&sv);

			// Parent space -> comp space. No-op for unparented layers.
			A_Matrix4 PM;
			if (BTS_ParentXform(suites, BTS_GetParent(suites, layerH), &t, &PM))
				BTS_XformPt(&PM, &px, &py, &pz);

			if (xP) *xP = px;
			if (yP) *yP = py;
			if (zP) *zP = pz;
		}
		suites.StreamSuite6()->AEGP_DisposeStream(streamH);
	}
	return err;
}

static A_Boolean BTS_IsMeshLayer (PF_InData *in_data, AEGP_LayerH layerH);

// Classify a layer into exactly one BTS_LT_* kind, and report the two ATTRIBUTE
// flags (guide / locked) that cut across kinds.
//
// Precedence matters: a null layer is technically an AV layer backed by a solid,
// and an adjustment layer is a solid too, so the distinguishing FLAGS are tested
// before falling back to the object type and finally to the source item. Order:
//   camera > light > null > adjustment > text > shape > precomp > solid > footage
// All three reads (flags / object type / source item) are cheap and already used
// elsewhere in this file; failures degrade to BTS_LT_FOOTAGE (the catch-all).
static A_long
BTS_LayerKind (PF_InData *in_data, AEGP_LayerH layerH,
               A_Boolean *isGuideP, A_Boolean *isLockedP)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);

	if (isGuideP)  *isGuideP  = FALSE;
	if (isLockedP) *isLockedP = FALSE;
	if (!layerH) return BTS_LT_FOOTAGE;

	AEGP_LayerFlags fl = 0;
	suites.LayerSuite9()->AEGP_GetLayerFlags(layerH, &fl);
	if (isGuideP)  *isGuideP  = (fl & AEGP_LayerFlag_GUIDE_LAYER) ? TRUE : FALSE;
	if (isLockedP) *isLockedP = (fl & AEGP_LayerFlag_LOCKED)      ? TRUE : FALSE;

	AEGP_ObjectType ot = AEGP_ObjectType_NONE;
	suites.LayerSuite9()->AEGP_GetLayerObjectType(layerH, &ot);
	if (ot == AEGP_ObjectType_CAMERA) return BTS_LT_CAMERA;
	if (ot == AEGP_ObjectType_LIGHT)  return BTS_LT_LIGHT;

	if (fl & AEGP_LayerFlag_NULL_LAYER)       return BTS_LT_NULL;
	if (fl & AEGP_LayerFlag_ADJUSTMENT_LAYER) return BTS_LT_ADJUST;

	if (ot == AEGP_ObjectType_TEXT)   return BTS_LT_TEXT;
	if (ot == AEGP_ObjectType_VECTOR) return BTS_LT_SHAPE;
	// An IMPORTED model (.glb) reports AEGP_ObjectType_3D_MODEL, item type
	// FOOTAGE and footage signature MODL, and its root stream is
	// "ADBE 3D Model Layer". AE exposes NO geometry extents for it anywhere in
	// its property tree, so it can never have a bounding box - confirmed by
	// dumping the full tree, not assumed.
	if (ot == AEGP_ObjectType_3D_MODEL) return BTS_LT_MODEL;
	// AE 2026 parametric primitives are a DIFFERENT animal: undocumented object
	// type, no source item, root match name "ADBE3D " - and they DO publish their
	// own dimension streams, so BTS_ReadBox builds a real 8-corner box for them.
	// Same-looking layers, permanently different capability = separate kinds.
	if (BTS_IsMeshLayer(in_data, layerH)) return BTS_LT_PRIMITIVE;

	// Remaining AV layers are told apart by their SOURCE ITEM: a comp source is a
	// precomp; a footage source whose signature is 'Soli' is a solid; anything
	// else (files, placeholders, 3D models) is plain footage.
	AEGP_ItemH itemH = NULL;
	if (suites.LayerSuite9()->AEGP_GetLayerSourceItem(layerH, &itemH) != PF_Err_NONE || !itemH)
		return BTS_LT_FOOTAGE;

	AEGP_ItemType it = AEGP_ItemType_NONE;
	suites.ItemSuite9()->AEGP_GetItemType(itemH, &it);
	if (it == AEGP_ItemType_COMP) return BTS_LT_PRECOMP;
	if (it == AEGP_ItemType_FOOTAGE) {
		AEGP_FootageH		footH = NULL;
		AEGP_FootageSignature sig = AEGP_FootageSignature_NONE;
		if (suites.FootageSuite5()->AEGP_GetMainFootageFromItem(itemH, &footH) == PF_Err_NONE && footH) {
			suites.FootageSuite5()->AEGP_GetFootageSignature(footH, &sig);
			if (sig == AEGP_FootageSignature_SOLID) return BTS_LT_SOLID;
		}
	}
	return BTS_LT_FOOTAGE;
}

// Is a layer "visible" at comp time t? TRUE when its video switch (eyeball) is on
// AND t falls within the layer's trimmed [in, out) span. Used by Window mode so
// overlays only appear for layers actually on-screen at the CTI. in/out (comp
// time) are also returned so the caller can clamp the motion-path window.
static A_Boolean
BTS_LayerVisibleAt (PF_InData *in_data, AEGP_CompH compH, A_long idx,
                    A_long tValue, PF_FpLong *inSecP, PF_FpLong *outSecP)
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_LayerH			layerH = NULL;

	ERR(suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, idx, &layerH));
	if (err || !layerH) return FALSE;

	AEGP_LayerFlags fl = 0;
	suites.LayerSuite9()->AEGP_GetLayerFlags(layerH, &fl);
	A_Boolean videoOn = (fl & AEGP_LayerFlag_VIDEO_ACTIVE) ? TRUE : FALSE;

	A_Time inT, durT;  AEFX_CLR_STRUCT(inT);  AEFX_CLR_STRUCT(durT);
	suites.LayerSuite9()->AEGP_GetLayerInPoint(layerH, AEGP_LTimeMode_CompTime, &inT);
	suites.LayerSuite9()->AEGP_GetLayerDuration(layerH, AEGP_LTimeMode_CompTime, &durT);

	PF_FpLong inSec  = (inT.scale  > 0) ? (PF_FpLong)inT.value  / (PF_FpLong)inT.scale  : 0.0;
	PF_FpLong durSec = (durT.scale > 0) ? (PF_FpLong)durT.value / (PF_FpLong)durT.scale : 0.0;

	// NORMALIZE the span. A negative Time Stretch (Time-Reverse Layer) can hand
	// back a negative duration, or an in-point that is really the later edge.
	// Unnormalized, every comparison below goes false, the layer is judged
	// never-visible, and Window mode drops it entirely - no marker, no box, no
	// path. Order the two edges and the rest of the math stops caring.
	PF_FpLong loSec = inSec, hiSec = inSec + durSec;
	if (hiSec < loSec) { PF_FpLong tmp = loSec;  loSec = hiSec;  hiSec = tmp; }

	if (inSecP)  *inSecP  = loSec;
	if (outSecP) *outSecP = hiSec;

	PF_FpLong tSec = (in_data->time_scale > 0) ? (PF_FpLong)tValue / (PF_FpLong)in_data->time_scale : 0.0;
	return videoOn && tSec >= loSec && tSec < hiSec;
}

/* =========================================================================
   AE 2026 parametric 3D primitives (cube / sphere / plane / torus / cone /
   cylinder).

   These layers do NOT report AEGP_ObjectType_3D_MODEL — they come back as an
   undocumented object type (7) with NO source item, so they used to fall into
   the footage catch-all and get a masked-bounds "box" that was really just the
   comp rect pinned at the anchor.

   What identifies them is their ROOT stream match name: "ADBE3D ..." (e.g.
   "ADBE3D ParametricMeshLayer"). And their real extent is readable: every mesh
   layer carries ALL SIX primitive option groups, but only the active one is
   un-HIDDEN, and its leaves hold the actual dimensions in layer units. From
   those we build a true 8-corner local-space box, transform it with the same
   layer->world matrix everything else uses, and project all 8 corners.
   ========================================================================= */

#define BTS_MESH_PREFIX		"ADBE3D "

// Extent formulas, per primitive family.
#define BTS_MFORM_BOX		0	// width, height, depth
#define BTS_MFORM_SPHERE	1	// radius
#define BTS_MFORM_PLANE		2	// width, length (flat: no Y extent)
#define BTS_MFORM_TORUS		3	// ring radius, pipe radius
#define BTS_MFORM_CONE		4	// top radius, bottom radius, height
#define BTS_MFORM_CYLINDER	5	// radius, height

typedef struct {
	const char	*group;			// match name of the primitive's option group
	const char	*s0, *s1, *s2;	// match names of its size leaves (NULL = unused)
	A_long		form;
} BTSMeshDef;

// Match names as reported by the diagnostics stream dump.
static const BTSMeshDef BTS_meshDefs[] = {
	{ "ADBE CubeMeshOptionsSGrp",     "ADBE CubeWidthStrm",       "ADBE CubeHeightStrm",     "ADBE CubeDepthStrm",  BTS_MFORM_BOX      },
	{ "ADBE SphereMeshOptionsSGrp",   "ADBE SphereRadiusStrm",    NULL,                      NULL,                  BTS_MFORM_SPHERE   },
	{ "ADBE PlaneMeshOptionsSGrp",    "ADBE PlaneWidthStrm",      "ADBE PlaneLengthStrm",    NULL,                  BTS_MFORM_PLANE    },
	{ "ADBE TorusMeshOptionsSGrp",    "ADBE TorusRingRadiusStrm", "ADBE TorusPipeRadiusStrm",NULL,                  BTS_MFORM_TORUS    },
	{ "ADBE ConeMeshOptionsSGrp",     "ADBE ConeTopRadiusStrm",   "ADBE ConeBottomRadiusStrm","ADBE ConeHeightStrm",BTS_MFORM_CONE     },
	{ "ADBE CylinderMeshOptionsSGrp", "ADBE CylinderRadiusStrm",  "ADBE CylinderHeightStrm", NULL,                  BTS_MFORM_CYLINDER },
};
#define BTS_NUM_MESHDEFS	(sizeof(BTS_meshDefs) / sizeof(BTS_meshDefs[0]))

// Read one numeric leaf out of a property group by match name.
static A_Boolean
BTS_ReadLeaf1D (PF_InData *in_data, AEGP_StreamRefH groupH, const char *matchName,
                const A_Time *tp, PF_FpLong *outP)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_StreamRefH		leafH = NULL;
	A_Boolean			got = FALSE;

	if (!matchName) return FALSE;
	if (suites.DynamicStreamSuite4()->AEGP_GetNewStreamRefByMatchname(
			S_bts_id, groupH, matchName, &leafH) != PF_Err_NONE || !leafH)
		return FALSE;

	AEGP_StreamValue2 sv;  AEFX_CLR_STRUCT(sv);
	if (suites.StreamSuite6()->AEGP_GetNewStreamValue(S_bts_id, leafH, AEGP_LTimeMode_CompTime,
													 tp, FALSE, &sv) == PF_Err_NONE) {
		*outP = sv.val.one_d;
		got = TRUE;
		suites.StreamSuite6()->AEGP_DisposeStreamValue(&sv);
	}
	suites.StreamSuite6()->AEGP_DisposeStream(leafH);
	return got;
}

// TRUE if this layer is one of AE's 3D mesh layers (root match name "ADBE3D ...").
static A_Boolean
BTS_IsMeshLayer (PF_InData *in_data, AEGP_LayerH layerH)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_StreamRefH		root = NULL;
	A_Boolean			isMesh = FALSE;

	if (!layerH) return FALSE;
	if (suites.DynamicStreamSuite4()->AEGP_GetNewStreamRefForLayer(S_bts_id, layerH, &root) != PF_Err_NONE
		|| !root)
		return FALSE;

	A_char mn[AEGP_MAX_STREAM_MATCH_NAME_SIZE];
	mn[0] = '\0';
	if (suites.DynamicStreamSuite4()->AEGP_GetMatchName(root, mn) == PF_Err_NONE)
		isMesh = (strncmp(mn, BTS_MESH_PREFIX, strlen(BTS_MESH_PREFIX)) == 0);

	suites.StreamSuite6()->AEGP_DisposeStream(root);
	return isMesh;
}

// Half-extents of a mesh layer's geometry in LAYER units, on the layer's own
// axes. Every mesh layer carries all six option groups; the ACTIVE primitive is
// the one whose group is not flagged HIDDEN. Height runs along Y (AE's up axis
// is -Y in world, but the box is symmetric so the sign does not matter); the
// flat primitives (plane, torus) lie in the XZ plane.
static A_Boolean
BTS_MeshExtents (PF_InData *in_data, AEGP_LayerH layerH, const A_Time *tp,
                 PF_FpLong *hxP, PF_FpLong *hyP, PF_FpLong *hzP)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_StreamRefH		root = NULL;
	A_Boolean			got = FALSE;

	if (!layerH) return FALSE;
	if (suites.DynamicStreamSuite4()->AEGP_GetNewStreamRefForLayer(S_bts_id, layerH, &root) != PF_Err_NONE
		|| !root)
		return FALSE;

	for (A_long d = 0; !got && d < (A_long)BTS_NUM_MESHDEFS; ++d) {
		const BTSMeshDef *md = &BTS_meshDefs[d];
		AEGP_StreamRefH grp = NULL;
		if (suites.DynamicStreamSuite4()->AEGP_GetNewStreamRefByMatchname(
				S_bts_id, root, md->group, &grp) != PF_Err_NONE || !grp)
			continue;

		AEGP_DynStreamFlags fl = 0;
		suites.DynamicStreamSuite4()->AEGP_GetDynamicStreamFlags(grp, &fl);
		if (fl & AEGP_DynStreamFlag_HIDDEN) {			// inactive primitive
			suites.StreamSuite6()->AEGP_DisposeStream(grp);
			continue;
		}

		PF_FpLong v0 = 0, v1 = 0, v2 = 0;
		A_Boolean ok = BTS_ReadLeaf1D(in_data, grp, md->s0, tp, &v0);
		if (ok && md->s1) ok = BTS_ReadLeaf1D(in_data, grp, md->s1, tp, &v1);
		if (ok && md->s2) ok = BTS_ReadLeaf1D(in_data, grp, md->s2, tp, &v2);

		if (ok) {
			switch (md->form) {
				case BTS_MFORM_BOX:				// width, height, depth
					*hxP = v0 * 0.5;  *hyP = v1 * 0.5;  *hzP = v2 * 0.5;  break;
				case BTS_MFORM_SPHERE:			// radius
					*hxP = *hyP = *hzP = v0;  break;
				case BTS_MFORM_PLANE:			// width, length; flat in XZ
					*hxP = v0 * 0.5;  *hyP = 0.0;  *hzP = v1 * 0.5;  break;
				case BTS_MFORM_TORUS:			// ring + pipe radius; ring in XZ
					*hxP = *hzP = v0 + v1;  *hyP = v1;  break;
				case BTS_MFORM_CONE:			// top r, bottom r, height
					*hxP = *hzP = (v0 > v1) ? v0 : v1;  *hyP = v2 * 0.5;  break;
				case BTS_MFORM_CYLINDER:		// radius, height
					*hxP = *hzP = v0;  *hyP = v1 * 0.5;  break;
				default:
					ok = FALSE;  break;
			}
			got = ok;
		}
		suites.StreamSuite6()->AEGP_DisposeStream(grp);
	}

	suites.StreamSuite6()->AEGP_DisposeStream(root);
	return got;
}

// Layer bounding box: the layer's rendered bounds (works for footage, solids,
// shapes AND text — unlike source-item dims), transformed to world/comp space
// via the layer-to-world matrix. Returns FALSE for empty bounds (cameras/lights).
// 2D read (z ignored); camera projection layers on top of this later.
static A_Boolean
BTS_ReadBox (PF_InData *in_data, AEGP_CompH compH, A_long idx,
             const BTSCam *cam, A_Boolean is3D,
             PF_FpLong *cornersX, PF_FpLong *cornersY, A_Boolean *is3DBoxP)
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_LayerH			layerH = NULL;

	if (is3DBoxP) *is3DBoxP = FALSE;
	ERR(suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, idx, &layerH));
	if (err || !layerH) return FALSE;

	// Only AV / text / vector layers have meaningful raster bounds.
	//  - camera / light: masked-bounds CRASHES on them.
	//  - 3D_MODEL (3D primitives / imported models): masked-bounds has no 2D answer
	//    for real geometry, so AE hands back a comp-sized rect pinned at the anchor
	//    — a box that is always wrong. The SDK exposes no geometry-extent API, so
	//    the honest move is no box at all; the marker and the 3D axis arrows (which
	//    come from the transform matrix, not from bounds) still describe the layer.
	//    See the "# 3dmodel-streams" section of the diagnostics CSV: it dumps each
	//    3D model layer's property tree so a real size/extent stream can be found
	//    if AE ever exposes one.
	AEGP_ObjectType ot = AEGP_ObjectType_NONE;
	suites.LayerSuite9()->AEGP_GetLayerObjectType(layerH, &ot);
	if (ot == AEGP_ObjectType_CAMERA || ot == AEGP_ObjectType_LIGHT ||
		ot == AEGP_ObjectType_3D_MODEL || ot == AEGP_ObjectType_NONE)
		return FALSE;

	// AE 2026 3D mesh layers: a real 8-corner box from the primitive's own
	// dimensions. Masked bounds is useless here (it returns the comp rect), so a
	// mesh layer either gets this box or none at all.
	if (BTS_IsMeshLayer(in_data, layerH)) {
		A_Time mt;  mt.value = in_data->current_time;  mt.scale = in_data->time_scale;
		PF_FpLong hx = 0, hy = 0, hz = 0;
		if (!BTS_MeshExtents(in_data, layerH, &mt, &hx, &hy, &hz)) return FALSE;

		A_Matrix4 MM;
		if (suites.LayerSuite9()->AEGP_GetLayerToWorldXform(layerH, &mt, &MM) != PF_Err_NONE)
			return FALSE;

		// Corner order: 0-3 = back face (-Z), 4-7 = front face (+Z), each wound
		// (-,-) (+,-) (+,+) (-,+). SmartRender's 12-edge table depends on this.
		static const PF_FpLong sx8[8] = { -1, 1, 1, -1, -1, 1, 1, -1 };
		static const PF_FpLong sy8[8] = { -1, -1, 1, 1, -1, -1, 1, 1 };
		static const PF_FpLong sz8[8] = { -1, -1, -1, -1, 1, 1, 1, 1 };
		for (A_long c = 0; c < 8; ++c) {
			PF_FpLong px = sx8[c] * hx, py = sy8[c] * hy, pz = sz8[c] * hz;
			PF_FpLong wx = px*MM.mat[0][0] + py*MM.mat[1][0] + pz*MM.mat[2][0] + MM.mat[3][0];
			PF_FpLong wy = px*MM.mat[0][1] + py*MM.mat[1][1] + pz*MM.mat[2][1] + MM.mat[3][1];
			PF_FpLong wz = px*MM.mat[0][2] + py*MM.mat[1][2] + pz*MM.mat[2][2] + MM.mat[3][2];
			BTS_ProjectPt(cam, TRUE, wx, wy, wz, &cornersX[c], &cornersY[c]);
		}
		if (is3DBoxP) *is3DBoxP = TRUE;
		return TRUE;
	}


	A_Time t;  t.value = in_data->current_time;  t.scale = in_data->time_scale;
	A_FloatRect rect;
	AEFX_CLR_STRUCT(rect);
	ERR(suites.LayerSuite9()->AEGP_GetLayerMaskedBounds(layerH, AEGP_LTimeMode_CompTime, &t, &rect));
	if (err || rect.right <= rect.left || rect.bottom <= rect.top) return FALSE;

	A_Matrix4 M;
	ERR(suites.LayerSuite9()->AEGP_GetLayerToWorldXform(layerH, &t, &M));
	if (err) return FALSE;

	// Bounds corners in layer space -> world (row-vector * matrix) -> screen px.
	PF_FpLong lx[4] = { rect.left, rect.right, rect.right, rect.left };
	PF_FpLong ly[4] = { rect.top,  rect.top,   rect.bottom, rect.bottom };
	for (A_long c = 0; c < 4; ++c) {
		PF_FpLong wx = lx[c]*M.mat[0][0] + ly[c]*M.mat[1][0] + M.mat[3][0];
		PF_FpLong wy = lx[c]*M.mat[0][1] + ly[c]*M.mat[1][1] + M.mat[3][1];
		PF_FpLong wz = lx[c]*M.mat[0][2] + ly[c]*M.mat[1][2] + M.mat[3][2];
		BTS_ProjectPt(cam, is3D, wx, wy, wz, &cornersX[c], &cornersY[c]);
	}
	return TRUE;
}

// 3D local-axis arrows: project the layer's anchor (origin) and the tips of its
// local X/Y/Z axes (rows of the layer->world matrix, length lenWorld). ex/ey hold
// 3. Only meaningful for 3D layers (caller gates on is3D).
static A_Boolean
BTS_ReadAxes (PF_InData *in_data, AEGP_CompH compH, A_long idx, const BTSCam *cam,
              PF_FpLong lenWorld, PF_FpLong *ox, PF_FpLong *oy, PF_FpLong *ex, PF_FpLong *ey)
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_LayerH			layerH = NULL;
	AEGP_StreamRefH		as = NULL;

	ERR(suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, idx, &layerH));
	if (err || !layerH) return FALSE;

	A_Time t;  t.value = in_data->current_time;  t.scale = in_data->time_scale;
	PF_FpLong ax = 0, ay = 0, az = 0;
	ERR(suites.StreamSuite6()->AEGP_GetNewLayerStream(S_bts_id, layerH, AEGP_LayerStream_ANCHORPOINT, &as));
	if (err || !as) return FALSE;
	AEGP_StreamValue2 sv;  AEFX_CLR_STRUCT(sv);
	if (suites.StreamSuite6()->AEGP_GetNewStreamValue(S_bts_id, as, AEGP_LTimeMode_CompTime, &t, FALSE, &sv) == PF_Err_NONE) {
		ax = sv.val.three_d.x;  ay = sv.val.three_d.y;  az = sv.val.three_d.z;
		suites.StreamSuite6()->AEGP_DisposeStreamValue(&sv);
	}
	suites.StreamSuite6()->AEGP_DisposeStream(as);

	A_Matrix4 M;
	ERR(suites.LayerSuite9()->AEGP_GetLayerToWorldXform(layerH, &t, &M));
	if (err) return FALSE;

	// Origin = anchor point transformed to world, then projected.
	PF_FpLong wx = ax*M.mat[0][0] + ay*M.mat[1][0] + az*M.mat[2][0] + M.mat[3][0];
	PF_FpLong wy = ax*M.mat[0][1] + ay*M.mat[1][1] + az*M.mat[2][1] + M.mat[3][1];
	PF_FpLong wz = ax*M.mat[0][2] + ay*M.mat[1][2] + az*M.mat[2][2] + M.mat[3][2];
	BTS_ProjectPt(cam, TRUE, wx, wy, wz, ox, oy);

	// Each local axis direction = a normalized row of the 3x3 linear part.
	// Per-axis sign to match AE's on-screen arrows at identity: X+ right (+),
	// Y+ up (world Y is down, so -), Z+ toward viewer (world Z is into-screen, so -).
	static const PF_FpLong sgn[3] = { 1.0, -1.0, -1.0 };
	for (A_long k = 0; k < 3; ++k) {
		PF_FpLong dx = M.mat[k][0], dy = M.mat[k][1], dz = M.mat[k][2];
		PF_FpLong len = sqrt(dx*dx + dy*dy + dz*dz);
		if (len > 1e-9) { dx /= len; dy /= len; dz /= len; }
		PF_FpLong s = sgn[k] * lenWorld;
		BTS_ProjectPt(cam, TRUE, wx + dx*s, wy + dy*s, wz + dz*s, &ex[k], &ey[k]);
	}
	return TRUE;
}

/* =========================================================================
   Stage 2 - motion-path sampling: frame-quantized, cached, density-scaled.

   Three cooperating ideas, all of which live in this block:

   1. QUANTIZE every sample to an ABSOLUTE sub-frame grid,
          t = k / (fps * BTS_SUBDIV) seconds,   k an integer counted from 0
      instead of "n evenly spaced points across whatever window I have now".
      Because k is absolute, nudging the CTI one frame re-asks AE for the SAME
      stream times it just evaluated, so AE's own stream cache answers most of
      them. The old code re-asked at a brand-new set of arbitrary sub-frame
      times on every single CTI position, hitting nothing.

   2. CACHE the samples per layer and re-evaluate only the newly exposed end
      when the window slides. Positions are cached in WORLD space, unprojected:
      an animated camera changes the projection every frame but not the
      positions, and projection is pure math we redo each PreRender anyway.
      The cache key is layer ID + grid step + a checksum over the Position
      keyframes (times and values), so any ordinary keyframe edit invalidates.
      Streams with fewer than 2 keyframes (expression-driven) have no
      meaningful checksum, so they are sampled fresh and never cached.

   3. SCALE DENSITY with window length. The grid step doubles until the window
      fits a sane point count, and a per-PreRender budget shared across all
      layers coarsens it further - 256 samples across a half-second window was
      absurd, and 64 layers doing it at once more so.
   ========================================================================= */

#define BTS_SUBDIV			8		// finest grid: 8 samples per comp frame
#define BTS_TARGET_PTS		128		// per-layer sample target before the budget bites
#define BTS_TOTAL_BUDGET	1536	// total samples across ALL layers, per PreRender

typedef struct {
	A_Boolean		used;
	AEGP_LayerIDVal	layerID;
	AEGP_CompH		compH;		// layer IDs are unique per COMP, not per project
	A_long			step;		// grid step, in 1/BTS_SUBDIV frame units
	A_long			kLo;		// absolute grid index of sample 0 (multiple of step)
	A_long			n;			// samples held
	PF_FpLong		sig;		// Position-keyframe checksum this was sampled at
	A_u_long		stamp;		// LRU
	PF_FpLong		wx[BTS_PATH_SAMPLES];	// WORLD space, unprojected
	PF_FpLong		wy[BTS_PATH_SAMPLES];
	PF_FpLong		wz[BTS_PATH_SAMPLES];
} BTSPathCache;

static BTSPathCache	S_pathCache[BTS_MAX_PATHS];
static A_u_long		S_pathStamp = 0;

/* PreRender runs on several render threads at once (MFR), so the cache needs a
   lock. Contention is per-layer-per-frame, i.e. nothing. */
static std::mutex	S_pathCacheMu;

/* Open a layer's Position stream. Caller disposes. Returns NULL on failure. */
static AEGP_StreamRefH
BTS_NewPosStream (PF_InData *in_data, AEGP_LayerH layerH)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_StreamRefH		streamH = NULL;
	if (!layerH) return NULL;
	if (suites.StreamSuite6()->AEGP_GetNewLayerStream(
			S_bts_id, layerH, AEGP_LayerStream_POSITION, &streamH) != PF_Err_NONE)
		return NULL;
	return streamH;
}

/* (4) Does this Position actually move? AEGP_IsStreamTimevarying counts
   expressions as well as keyframes, so a static layer costs one call here and
   then no sampling, no keyframe reads and no window math at all. */
static A_Boolean
BTS_PosIsAnimated (PF_InData *in_data, AEGP_StreamRefH streamH)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	A_Boolean			tv = FALSE;
	if (!streamH) return FALSE;
	if (suites.StreamSuite6()->AEGP_IsStreamTimevarying(streamH, &tv) != PF_Err_NONE) return FALSE;
	return tv;
}

/* Checksum over a Position stream's keyframes (times + values) - the cache's
   invalidation key. Cheap: keyframe counts are small (tens), versus the
   hundreds of stream evaluations it protects. Returns 0 for streams with < 2
   keyframes, which means "not cacheable". */
static PF_FpLong
BTS_PosSig (AEGP_SuiteHandler &suites, AEGP_StreamRefH streamH)
{
	A_long nk = 0;
	if (suites.KeyframeSuite5()->AEGP_GetStreamNumKFs(streamH, &nk) != PF_Err_NONE || nk < 2)
		return 0.0;

	PF_FpLong sig = (PF_FpLong)nk * 7919.0;
	for (A_long j = 0; j < nk; ++j) {
		A_Time kt;  AEFX_CLR_STRUCT(kt);
		if (suites.KeyframeSuite5()->AEGP_GetKeyframeTime(streamH, j, AEGP_LTimeMode_CompTime, &kt) == PF_Err_NONE)
			sig += (PF_FpLong)kt.value * (PF_FpLong)(j + 11);
		AEGP_StreamValue2 kv;  AEFX_CLR_STRUCT(kv);
		if (suites.KeyframeSuite5()->AEGP_GetNewKeyframeValue(S_bts_id, streamH, j, &kv) == PF_Err_NONE) {
			sig += kv.val.three_d.x * (PF_FpLong)(j + 2)
				 + kv.val.three_d.y * (PF_FpLong)(j + 3)
				 + kv.val.three_d.z * (PF_FpLong)(j + 5);
			suites.StreamSuite6()->AEGP_DisposeStreamValue(&kv);
		}
	}
	return (sig == 0.0) ? 1.0 : sig;	// never collide with the "not cacheable" 0
}

/* Evaluate Position at absolute grid index k. The tick value is a pure function
   of k (and the comp's fps / time scale), which is the whole point: the same k
   always asks AE for the same time. */
static A_Boolean
BTS_EvalGrid (AEGP_SuiteHandler &suites, AEGP_StreamRefH streamH,
              AEGP_LayerH parentH, A_long k,
              A_FpLong fps, A_long scale,
              PF_FpLong *x, PF_FpLong *y, PF_FpLong *z)
{
	A_Time t;
	t.scale = scale;
	t.value = (A_long)floor((A_FpLong)k * (A_FpLong)scale / (fps * (A_FpLong)BTS_SUBDIV) + 0.5);

	AEGP_StreamValue2 sv;  AEFX_CLR_STRUCT(sv);
	if (suites.StreamSuite6()->AEGP_GetNewStreamValue(
			S_bts_id, streamH, AEGP_LTimeMode_CompTime, &t, FALSE, &sv) != PF_Err_NONE)
		return FALSE;
	*x = sv.val.three_d.x;  *y = sv.val.three_d.y;  *z = sv.val.three_d.z;
	suites.StreamSuite6()->AEGP_DisposeStreamValue(&sv);

	// Parent space -> comp space, at THIS sample time (the parent may itself
	// be animated). No-op for unparented layers.
	A_Matrix4 PM;
	if (BTS_ParentXform(suites, parentH, &t, &PM)) BTS_XformPt(&PM, x, y, z);
	return TRUE;
}

/* Sample world-space Position at grid indices kLo, kLo+step, ... <= kHi,
   reusing whatever this layer's cache slot already holds. Returns the number of
   samples written into wx/wy/wz (0 if fewer than 2 could be read). */
static A_long
BTS_SampleGrid (PF_InData *in_data, AEGP_StreamRefH streamH, AEGP_LayerH parentH,
                AEGP_CompH compH, AEGP_LayerIDVal layerID,
                PF_FpLong sig, A_long kLo, A_long kHi, A_long step,
                A_FpLong fps, A_long scale,
                PF_FpLong *wx, PF_FpLong *wy, PF_FpLong *wz)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);

	A_long n = (kHi - kLo) / step + 1;
	if (n < 2) return 0;
	if (n > BTS_PATH_SAMPLES) n = BTS_PATH_SAMPLES;

	// Not cacheable (expression-driven): straight sample, no bookkeeping.
	if (sig == 0.0) {
		A_long w = 0;
		for (A_long i = 0; i < n; ++i)
			if (BTS_EvalGrid(suites, streamH, parentH, kLo + i * step, fps, scale,
							 &wx[w], &wy[w], &wz[w]))
				w++;
		return (w > 1) ? w : 0;
	}

	std::lock_guard<std::mutex> guard(S_pathCacheMu);

	// Find this layer's slot, or claim the least-recently-used one.
	BTSPathCache *c = NULL;
	for (A_long s = 0; s < BTS_MAX_PATHS; ++s)
		if (S_pathCache[s].used && S_pathCache[s].layerID == layerID
			&& S_pathCache[s].compH == compH) { c = &S_pathCache[s]; break; }
	if (!c) {
		c = &S_pathCache[0];
		for (A_long s = 0; s < BTS_MAX_PATHS; ++s) {
			if (!S_pathCache[s].used) { c = &S_pathCache[s]; break; }
			if (S_pathCache[s].stamp < c->stamp) c = &S_pathCache[s];
		}
		c->used = FALSE;
	}

	// A different grid step or an edited stream throws the whole slot away.
	A_Boolean reuse = c->used && c->step == step && c->sig == sig && c->n > 0;

	PF_FpLong	nx[BTS_PATH_SAMPLES], ny[BTS_PATH_SAMPLES], nz[BTS_PATH_SAMPLES];
	A_Boolean	got[BTS_PATH_SAMPLES];
	memset(got, 0, sizeof(got));

	// Slide: copy the overlapping grid indices straight across. Both windows are
	// aligned to the same absolute grid at the same step, so indices line up
	// exactly - no resampling, no interpolation.
	if (reuse) {
		A_long haveLo = c->kLo, haveHi = c->kLo + (c->n - 1) * step;
		A_long ovLo = (kLo > haveLo) ? kLo : haveLo;
		A_long ovHi = (kHi < haveHi) ? kHi : haveHi;
		for (A_long kk = ovLo; kk <= ovHi; kk += step) {
			A_long src = (kk - haveLo) / step, dst = (kk - kLo) / step;
			if (src < 0 || src >= c->n || dst < 0 || dst >= n) continue;
			nx[dst] = c->wx[src];  ny[dst] = c->wy[src];  nz[dst] = c->wz[src];
			got[dst] = TRUE;
		}
	}

	// Only the newly exposed end(s) actually cost an evaluation.
	for (A_long i = 0; i < n; ++i)
		if (!got[i] && BTS_EvalGrid(suites, streamH, parentH, kLo + i * step, fps, scale,
									&nx[i], &ny[i], &nz[i]))
			got[i] = TRUE;

	A_long w = 0;
	for (A_long i = 0; i < n; ++i)
		if (got[i]) { nx[w] = nx[i];  ny[w] = ny[i];  nz[w] = nz[i];  w++; }

	if (w == n) {	// a hole would misalign the grid, so only a full run is stored
		c->used = TRUE;  c->layerID = layerID;  c->compH = compH;  c->step = step;
		c->kLo = kLo;    c->n = n;             c->sig = sig;
		c->stamp = ++S_pathStamp;
		memcpy(c->wx, nx, (size_t)n * sizeof(PF_FpLong));
		memcpy(c->wy, ny, (size_t)n * sizeof(PF_FpLong));
		memcpy(c->wz, nz, (size_t)n * sizeof(PF_FpLong));
	} else {
		c->used = FALSE;
	}

	if (w < 2) return 0;
	memcpy(wx, nx, (size_t)w * sizeof(PF_FpLong));
	memcpy(wy, ny, (size_t)w * sizeof(PF_FpLong));
	memcpy(wz, nz, (size_t)w * sizeof(PF_FpLong));
	return w;
}

// First & last Position keyframe comp times (sec). Returns the keyframe count;
// firstSec/lastSec valid only when count >= 1. Lets Window mode clamp the path to
// where the layer actually MOVES, so past the last key the trail erodes (no static
// tail dot lingering ahead of / behind the motion).
static A_long
BTS_KfRange (AEGP_SuiteHandler &suites, AEGP_StreamRefH streamH,
             PF_FpLong *firstSec, PF_FpLong *lastSec)
{
	A_long nk = 0;
	if (!streamH) return 0;
	if (suites.KeyframeSuite5()->AEGP_GetStreamNumKFs(streamH, &nk) != PF_Err_NONE) return 0;
	// Scan for the MIN and MAX comp time rather than trusting keyframe 0 and
	// nk-1 to be the earliest and latest. Time Stretch with a NEGATIVE value
	// (Time-Reverse Layer) flips the comp-time order of the keyframes, so the
	// old endpoint read handed back first > last. Every caller then computed an
	// empty window (we <= ws) and silently dropped the motion path, leaving a
	// reversed layer with a marker and nothing else. A scan is nk calls, which
	// is nothing next to the sampling it gates, and it cannot be fooled by any
	// ordering.
	PF_FpLong lo = 0.0, hi = 0.0;
	A_Boolean seen = FALSE;		// NOT "j == 0": that keyframe's read can fail
	for (A_long j = 0; j < nk; ++j) {
		A_Time kt;  AEFX_CLR_STRUCT(kt);
		if (suites.KeyframeSuite5()->AEGP_GetKeyframeTime(streamH, j, AEGP_LTimeMode_CompTime, &kt) != PF_Err_NONE)
			continue;
		PF_FpLong sec = (kt.scale > 0) ? (PF_FpLong)kt.value / (PF_FpLong)kt.scale : 0.0;
		if (!seen) { lo = hi = sec;  seen = TRUE; }
		else if (sec < lo) lo = sec;
		else if (sec > hi) hi = sec;
	}
	if (!seen) return 0;		// no readable keyframe time = no usable range
	if (firstSec) *firstSec = lo;
	if (lastSec)  *lastSec  = hi;
	return nk;
}

// Same, addressed by comp layer index (opens/closes its own stream). Used by the
// diagnostics dump, which has no stream ref of its own to hand down.
static A_long
BTS_LayerKfRange (PF_InData *in_data, AEGP_CompH compH, A_long idx,
                  PF_FpLong *firstSec, PF_FpLong *lastSec)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_LayerH			layerH = NULL;

	if (suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, idx, &layerH) != PF_Err_NONE || !layerH)
		return 0;
	AEGP_StreamRefH streamH = BTS_NewPosStream(in_data, layerH);
	if (!streamH) return 0;
	A_long nk = BTS_KfRange(suites, streamH, firstSec, lastSec);
	suites.StreamSuite6()->AEGP_DisposeStream(streamH);
	return nk;
}

// Read a layer's Position keyframes + spatial-tangent handle endpoints (comp px).
// Returns the number of keyframes written; 0 if none (e.g. expression-driven).
// Only keyframes whose comp time falls in [winStartSec, winEndSec] are kept (pass
// a huge range to keep them all) - so kf dots + tangent handles honor Window mode.
static A_long
BTS_ReadKeyframes (AEGP_SuiteHandler &suites, AEGP_StreamRefH streamH,
                   AEGP_LayerH parentH, const BTSCam *cam, A_Boolean is3D,
                   PF_FpLong *kx, PF_FpLong *ky,
                   PF_FpLong *inx, PF_FpLong *iny,
                   PF_FpLong *outx, PF_FpLong *outy, A_long maxKf,
                   PF_FpLong winStartSec, PF_FpLong winEndSec)
{
	A_long written = 0, nk = 0;

	if (!streamH) return 0;
	if (suites.KeyframeSuite5()->AEGP_GetStreamNumKFs(streamH, &nk) != PF_Err_NONE) return 0;

	for (A_long j = 0; j < nk && written < maxKf; ++j) {
		// Skip keyframes outside the window (comp time).
		A_Time kt;  AEFX_CLR_STRUCT(kt);
		A_Boolean haveKt = FALSE;
		if (suites.KeyframeSuite5()->AEGP_GetKeyframeTime(streamH, j, AEGP_LTimeMode_CompTime, &kt) == PF_Err_NONE) {
			PF_FpLong ktSec = (kt.scale > 0) ? (PF_FpLong)kt.value / (PF_FpLong)kt.scale : 0.0;
			if (ktSec < winStartSec || ktSec > winEndSec) continue;
			haveKt = TRUE;
		}
		// Parent transform AT THIS KEYFRAME'S TIME - the keyframe value and both
		// tangent handles live in parent space, so all three go through it.
		A_Matrix4 PM;
		A_Boolean xf = haveKt && BTS_ParentXform(suites, parentH, &kt, &PM);
		AEGP_StreamValue2 kv;  AEFX_CLR_STRUCT(kv);
		if (suites.KeyframeSuite5()->AEGP_GetNewKeyframeValue(S_bts_id, streamH, j, &kv) == PF_Err_NONE) {
			PF_FpLong px = kv.val.three_d.x, py = kv.val.three_d.y, pz = kv.val.three_d.z;
			suites.StreamSuite6()->AEGP_DisposeStreamValue(&kv);
			PF_FpLong wpx = px, wpy = py, wpz = pz;
			if (xf) BTS_XformPt(&PM, &wpx, &wpy, &wpz);
			BTS_ProjectPt(cam, is3D, wpx, wpy, wpz, &kx[written], &ky[written]);

			// Spatial tangents are vectors from the keyframe (0 if unavailable).
			// Handle endpoints are projected in world space.
			AEGP_StreamValue2 it, ot;  AEFX_CLR_STRUCT(it);  AEFX_CLR_STRUCT(ot);
			if (suites.KeyframeSuite5()->AEGP_GetNewKeyframeSpatialTangents(
					S_bts_id, streamH, j, &it, &ot) == PF_Err_NONE) {
				// Tangents are offsets in the SAME parent space as the value, so
				// transform the endpoint, not the direction.
				PF_FpLong ix = px+it.val.three_d.x, iy = py+it.val.three_d.y, iz = pz+it.val.three_d.z;
				PF_FpLong ox = px+ot.val.three_d.x, oy = py+ot.val.three_d.y, oz = pz+ot.val.three_d.z;
				if (xf) { BTS_XformPt(&PM, &ix, &iy, &iz);  BTS_XformPt(&PM, &ox, &oy, &oz); }
				BTS_ProjectPt(cam, is3D, ix, iy, iz, &inx[written], &iny[written]);
				BTS_ProjectPt(cam, is3D, ox, oy, oz, &outx[written], &outy[written]);
				suites.StreamSuite6()->AEGP_DisposeStreamValue(&it);
				suites.StreamSuite6()->AEGP_DisposeStreamValue(&ot);
			} else {
				inx[written] = kx[written];  iny[written] = ky[written];
				outx[written] = kx[written]; outy[written] = ky[written];
			}
			written++;
		}
	}
	return written;
}

// Just the keyframe count on a layer's Position stream (diagnostic).
static A_long
BTS_LayerNumKFs (PF_InData *in_data, AEGP_CompH compH, A_long idx)
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	AEGP_LayerH			layerH = NULL;
	AEGP_StreamRefH		streamH = NULL;
	A_long				nk = 0;

	ERR(suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, idx, &layerH));
	if (err || !layerH) return -1;
	ERR(suites.StreamSuite6()->AEGP_GetNewLayerStream(
			S_bts_id, layerH, AEGP_LayerStream_POSITION, &streamH));
	if (err || !streamH) return -1;
	ERR(suites.KeyframeSuite5()->AEGP_GetStreamNumKFs(streamH, &nk));
	suites.StreamSuite6()->AEGP_DisposeStream(streamH);
	return err ? -1 : nk;
}

/* Raw identification signals for one layer, exactly as the AEGP suites report
   them. Written verbatim into the CSV so an unrecognized layer type (AE 2026's
   3D primitives do NOT report AEGP_ObjectType_3D_MODEL — they fall into the AV/
   footage catch-all) can be identified from real numbers instead of guessed at.
   footage_sig is a 4-char code packed into an A_long ('Soli' for solids), so
   print it both as an integer and as its FourCC. */
static void
BTS_ProbeLayer (PF_InData *in_data, AEGP_LayerH layerH, A_long *otP, A_long *itP,
                A_long *sigP, char *sigStr, char *srcName, A_long srcNameLen)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);

	*otP = -99;  *itP = -99;  *sigP = 0;
	sigStr[0] = '\0';  srcName[0] = '\0';
	if (!layerH) return;

	AEGP_ObjectType ot = AEGP_ObjectType_NONE;
	suites.LayerSuite9()->AEGP_GetLayerObjectType(layerH, &ot);
	*otP = (A_long)ot;

	AEGP_ItemH itemH = NULL;
	if (suites.LayerSuite9()->AEGP_GetLayerSourceItem(layerH, &itemH) != PF_Err_NONE || !itemH) return;

	AEGP_ItemType it = AEGP_ItemType_NONE;
	suites.ItemSuite9()->AEGP_GetItemType(itemH, &it);
	*itP = (A_long)it;

	AEGP_MemHandle nameH = NULL;
	if (suites.ItemSuite9()->AEGP_GetItemName(S_bts_id, itemH, &nameH) == PF_Err_NONE)
		BTS_ReadName(suites, nameH, srcName, srcNameLen);

	if (it == AEGP_ItemType_FOOTAGE) {
		AEGP_FootageH			footH = NULL;
		AEGP_FootageSignature	sig = AEGP_FootageSignature_NONE;
		if (suites.FootageSuite5()->AEGP_GetMainFootageFromItem(itemH, &footH) == PF_Err_NONE && footH) {
			suites.FootageSuite5()->AEGP_GetFootageSignature(footH, &sig);
			*sigP = (A_long)sig;
			for (A_long b = 0; b < 4; ++b) {		// unpack the FourCC, printable chars only
				char ch = (char)(((A_long)sig >> (8 * (3 - b))) & 0xFF);
				sigStr[b] = (ch >= 32 && ch < 127) ? ch : '.';
			}
			sigStr[4] = '\0';
		}
	}
}

/* -------------------------------------------------------------------------
   Diagnostic: recursive property-tree dump for 3D MODEL layers.

   AE gives no geometry-extent API for 3D primitives / imported models, which is
   why BTS_ReadBox refuses to draw a box for them (masked-bounds returns a
   comp-sized rect at the anchor). If a usable size/extent property exists at
   all, it lives somewhere in the layer's dynamic property tree — so dump that
   tree (match name, display name, and leaf value) into the diagnostics CSV and
   read it there instead of guessing.
   ------------------------------------------------------------------------- */
static void
BTS_DumpStreamTree (PF_InData *in_data, FILE *fp, AEGP_StreamRefH sref,
                    A_long layerIdx, A_long depth)
{
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	if (!sref || depth > 4) return;

	A_char matchName[AEGP_MAX_STREAM_MATCH_NAME_SIZE];
	matchName[0] = '\0';
	suites.DynamicStreamSuite4()->AEGP_GetMatchName(sref, matchName);

	char dispName[128];
	AEGP_MemHandle nameH = NULL;
	if (suites.StreamSuite6()->AEGP_GetStreamName(S_bts_id, sref, TRUE, &nameH) == PF_Err_NONE)
		BTS_ReadName(suites, nameH, dispName, sizeof(dispName));
	else
		dispName[0] = '\0';

	AEGP_StreamGroupingType gt = AEGP_StreamGroupingType_NONE;
	suites.DynamicStreamSuite4()->AEGP_GetStreamGroupingType(sref, &gt);

	if (gt == AEGP_StreamGroupingType_LEAF) {
		// Leaf: print whatever numeric value it carries, so a "Size"-like
		// property can be recognized by its numbers as well as its name.
		AEGP_StreamType st = AEGP_StreamType_NO_DATA;
		suites.StreamSuite6()->AEGP_GetStreamType(sref, &st);
		A_Time t;  t.value = in_data->current_time;  t.scale = in_data->time_scale;
		AEGP_StreamValue2 sv;  AEFX_CLR_STRUCT(sv);
		char valBuf[96];  valBuf[0] = '\0';
		if (suites.StreamSuite6()->AEGP_GetNewStreamValue(S_bts_id, sref, AEGP_LTimeMode_CompTime,
														 &t, FALSE, &sv) == PF_Err_NONE) {
			switch (st) {
				case AEGP_StreamType_OneD:
					suites.ANSICallbacksSuite1()->sprintf(valBuf, "%.4f", sv.val.one_d); break;
				case AEGP_StreamType_TwoD:
				case AEGP_StreamType_TwoD_SPATIAL:
					suites.ANSICallbacksSuite1()->sprintf(valBuf, "%.4f %.4f", sv.val.two_d.x, sv.val.two_d.y); break;
				case AEGP_StreamType_ThreeD:
				case AEGP_StreamType_ThreeD_SPATIAL:
					suites.ANSICallbacksSuite1()->sprintf(valBuf, "%.4f %.4f %.4f",
						sv.val.three_d.x, sv.val.three_d.y, sv.val.three_d.z); break;
				default: break;
			}
			suites.StreamSuite6()->AEGP_DisposeStreamValue(&sv);
		}
		fprintf(fp, "# %ld,%ld,LEAF,%s,%s,type=%d,%s\n",
				(long)layerIdx, (long)depth, matchName, dispName, (int)st, valBuf);
		return;
	}

	A_long n = 0;
	suites.DynamicStreamSuite4()->AEGP_GetNumStreamsInGroup(sref, &n);
	fprintf(fp, "# %ld,%ld,GROUP,%s,%s,children=%ld\n",
			(long)layerIdx, (long)depth, matchName, dispName, (long)n);

	for (A_long i = 0; i < n; ++i) {
		AEGP_StreamRefH child = NULL;
		if (suites.DynamicStreamSuite4()->AEGP_GetNewStreamRefByIndex(S_bts_id, sref, i, &child) == PF_Err_NONE
			&& child) {
			BTS_DumpStreamTree(in_data, fp, child, layerIdx, depth + 1);
			suites.StreamSuite6()->AEGP_DisposeStream(child);
		}
	}
}

/* =========================================================================
   Diagnostics dump (main thread) — full per-layer table
   ========================================================================= */

static PF_Err
DumpDiagnostics (PF_InData *in_data, PF_OutData *out_data, PF_ParamDef *params[])
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);

	PF_FpLong dsx = (PF_FpLong)in_data->downsample_x.num / (PF_FpLong)in_data->downsample_x.den;
	PF_FpLong dsy = (PF_FpLong)in_data->downsample_y.num / (PF_FpLong)in_data->downsample_y.den;

	AEGP_ErrReportState errState;
	suites.UtilitySuite3()->AEGP_StartQuietErrors(&errState);

	AEGP_CompH compH = NULL;
	A_long myIdx = 0, num = 0;
	ERR(BTS_GetContext(in_data, &compH, &myIdx, &num));

	FILE *fp = fopen(BTS_DIAG_PATH, "w");
	if (!fp) {
		suites.UtilitySuite3()->AEGP_EndQuietErrors(FALSE, &errState);
		suites.ANSICallbacksSuite1()->sprintf(out_data->return_msg,
			"BTS Overlay: could not write %s", BTS_DIAG_PATH);
		return err;
	}

	time_t now = time(NULL);
	char stamp[64];
	strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", localtime(&now));

	fprintf(fp, "# BTS Overlay diagnostics - build %d, %s\n", BUILD_VERSION, stamp);
	fprintf(fp, "# aegp_plugin_id=%ld view=%ld my_layer_index=%ld num_layers=%ld\n",
			(long)S_bts_id, (long)params[BTS_VIEW]->u.pd.value, (long)myIdx, (long)num);
	fprintf(fp, "# env: current_time=%ld time_scale=%ld width=%ld height=%ld downsample=%.4f,%.4f\n",
			(long)in_data->current_time, (long)in_data->time_scale,
			(long)in_data->width, (long)in_data->height, dsx, dsy);
	// Time Range state, so the window math can be checked frame-by-frame.
	A_Boolean windowMode = (params[BTS_VISIBILITY]->u.pd.value == BTS_VIS_WINDOW);
	PF_FpLong beforeSec  = params[BTS_WIN_START]->u.fs_d.value;
	PF_FpLong afterSec   = params[BTS_WIN_LEN]->u.fs_d.value;
	PF_FpLong tSec = (in_data->time_scale > 0)
					 ? (PF_FpLong)in_data->current_time / (PF_FpLong)in_data->time_scale : 0.0;
	fprintf(fp, "# time_range: mode=%s before=%.3f after=%.3f cti_sec=%.4f\n",
			windowMode ? "WINDOW" : "WHOLE", beforeSec, afterSec, tSec);
	fprintf(fp, "index,layer_name,kind,obj_type,item_type,footage_sig,sig_fourcc,src_name,guide,locked,type_shown,is_3d,num_kfs,pos_comp_x,pos_comp_y,pos_comp_z,our_screen_x,our_screen_y,"
				"visible_at_cti,in_sec,out_sec,kf_first,kf_last,win_start,win_end,win_len,drawn,"
				"stretch,raw_in_sec,raw_dur_sec,pos_read_err,parented\n");

	A_long cap = (num < BTS_MAX_LAYERS) ? num : BTS_MAX_LAYERS;
	for (A_long i = 0; !err && i < cap; ++i) {
		PF_FpLong x = 0, y = 0, z = 0;
		A_Boolean is3d = FALSE;
		char name[128];
		PF_Err e = BTS_ReadLayer(in_data, compH, i, &x, &y, &z, &is3d, name, sizeof(name));
		if (e) { name[0] = '\0'; }

		// Raw, UNNORMALIZED timing straight from AE, the stretch ratio, and
		// whether the Position read itself errored. Together these separate
		// "skipped by the visibility test" from "BTS_ReadLayer failed" - the
		// fork an empty picture cannot tell us apart.
		AEGP_LayerH tLayerH = NULL;
		suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, i, &tLayerH);
		A_Ratio stretch;  stretch.num = 0;  stretch.den = 1;
		A_Time rawIn, rawDur;  AEFX_CLR_STRUCT(rawIn);  AEFX_CLR_STRUCT(rawDur);
		A_Boolean parented = FALSE;
		if (tLayerH) {
			suites.LayerSuite9()->AEGP_GetLayerStretch(tLayerH, &stretch);
			suites.LayerSuite9()->AEGP_GetLayerInPoint(tLayerH, AEGP_LTimeMode_CompTime, &rawIn);
			suites.LayerSuite9()->AEGP_GetLayerDuration(tLayerH, AEGP_LTimeMode_CompTime, &rawDur);
			parented = (BTS_GetParent(suites, tLayerH) != NULL);
		}
		PF_FpLong stretchF  = (stretch.den != 0) ? (PF_FpLong)stretch.num / (PF_FpLong)stretch.den : 0.0;
		PF_FpLong rawInSec  = (rawIn.scale  > 0) ? (PF_FpLong)rawIn.value  / (PF_FpLong)rawIn.scale  : 0.0;
		PF_FpLong rawDurSec = (rawDur.scale > 0) ? (PF_FpLong)rawDur.value / (PF_FpLong)rawDur.scale : 0.0;		// skip unreadable layers, keep going
		A_long nkf = BTS_LayerNumKFs(in_data, compH, i);

		// Layer-type classification, so the discriminators can be checked numerically.
		AEGP_LayerH kLayerH = NULL;
		suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, i, &kLayerH);
		A_Boolean isGuide = FALSE, isLocked = FALSE;
		A_long kind = BTS_LayerKind(in_data, kLayerH, &isGuide, &isLocked);
		A_long otRaw = 0, itRaw = 0, sigRaw = 0;
		char sigStr[8], srcName[128];
		BTS_ProbeLayer(in_data, kLayerH, &otRaw, &itRaw, &sigRaw, sigStr, srcName, sizeof(srcName));
		A_Boolean typeShown = params[BTS_SHOW_CAMERAS + 2 * kind]->u.bd.value
					  && !(isGuide  && !params[BTS_SHOW_GUIDES]->u.bd.value)
					  && !(isLocked && !params[BTS_SHOW_LOCKED]->u.bd.value);

		// Replay EXACTLY the PreRender window math for this layer.
		PF_FpLong inSec = 0, outSec = 0;
		A_Boolean vis = BTS_LayerVisibleAt(in_data, compH, i, in_data->current_time, &inSec, &outSec);
		PF_FpLong kfFirst = 0, kfLast = 0;
		A_Boolean haveKf = (BTS_LayerKfRange(in_data, compH, i, &kfFirst, &kfLast) >= 2);
		PF_FpLong ws = -1e18, we = 1e18;
		if (haveKf) { ws = kfFirst; we = kfLast; }
		if (windowMode) {
			PF_FpLong cs = tSec - beforeSec, ce = tSec + afterSec;
			if (cs > ws) ws = cs;
			if (ce < we) we = ce;
			if (ws < inSec)  ws = inSec;
			if (we > outSec) we = outSec;
		}
		A_Boolean bounded = (haveKf || windowMode);
		A_Boolean drawn = bounded ? (we > ws) : TRUE;

		// Main-thread mapping uses origin 0 (no render tile here).
		fprintf(fp, "%ld,%s,%s,%ld,%ld,%ld,%s,%s,%d,%d,%d,%d,%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%ld,%d\n",
				(long)i, name, BTS_kindName[kind],
				(long)otRaw, (long)itRaw, (long)sigRaw, sigStr, srcName,
				isGuide ? 1 : 0, isLocked ? 1 : 0,
				typeShown ? 1 : 0, is3d ? 1 : 0, (long)nkf, x, y, z, x * dsx, y * dsy,
				vis ? 1 : 0, inSec, outSec,
				haveKf ? kfFirst : -1.0, haveKf ? kfLast : -1.0,
				bounded ? ws : -1.0, bounded ? we : -1.0,
				bounded ? (we - ws) : -1.0, drawn ? 1 : 0,
				stretchF, rawInSec, rawDurSec, (long)e, parented ? 1 : 0);
	}
	// 3D model layers: property-tree dump (see BTS_DumpStreamTree). Only these,
	// because a full tree for every layer would swamp the file.
	// Property tree for 3D-model layers AND for anything that landed in the
	// FOOTAGE catch-all - that is where an unrecognized new layer type hides.
	fprintf(fp, "# unknown-layer-streams: layer_index,depth,kind,match_name,display_name,value\n");
	for (A_long i = 0; i < cap; ++i) {
		AEGP_LayerH layerH = NULL;
		suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, i, &layerH);
		if (!layerH) continue;
		AEGP_ObjectType ot = AEGP_ObjectType_NONE;
		suites.LayerSuite9()->AEGP_GetLayerObjectType(layerH, &ot);
		A_Boolean gu = FALSE, lo = FALSE;
		A_long k = BTS_LayerKind(in_data, layerH, &gu, &lo);
		if (ot != AEGP_ObjectType_3D_MODEL && k != BTS_LT_FOOTAGE) continue;

		AEGP_StreamRefH root = NULL;
		if (suites.DynamicStreamSuite4()->AEGP_GetNewStreamRefForLayer(S_bts_id, layerH, &root) == PF_Err_NONE
			&& root) {
			BTS_DumpStreamTree(in_data, fp, root, i, 0);
			suites.StreamSuite6()->AEGP_DisposeStream(root);
		}
	}

	fclose(fp);
	suites.UtilitySuite3()->AEGP_EndQuietErrors(FALSE, &errState);

	suites.ANSICallbacksSuite1()->sprintf(out_data->return_msg,
		"BTS Overlay: %ld layers written to\r%s", (long)num, BTS_DIAG_PATH);
	return err;
}

/* =========================================================================
   Conditional parameter UI

   The effect carries ~50 params and most of them only matter when some master
   toggle above them is on. AE greys a control out when PF_PUI_DISABLED is set
   on it via PF_UpdateParamUI; getting the chance to do that needs
   PF_OutFlag_SEND_UPDATE_PARAMS_UI at GLOBAL_SETUP (mirrored in the PiPL, or
   AE refuses to load us), which makes AE send PF_Cmd_UPDATE_PARAMS_UI whenever
   a param value changes.

   The rules below MIRROR the branch nesting in SmartRender exactly: a control
   is greyed precisely when it cannot affect a pixel. Two of those relationships
   are easy to guess wrong, so they are spelled out at their rules -
   Show Paths really is a master, Show Handles really is not a child of boxes.

   Greying rather than hiding (PF_PUI_INVISIBLE) is deliberate: the panel keeps
   a stable shape, so you can still see WHAT the effect offers and which switch
   turns it back on.
   ========================================================================= */

// Grey out (or restore) one param. Only calls AE when the state actually
// flips - pushing every param on every change makes the Effect Controls panel
// visibly flicker and rebuild.
static PF_Err
BTS_SetEnabled (PF_InData *in_data, PF_ParamDef *params[], A_long idx, A_Boolean on)
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);

	A_Boolean disabledNow = (params[idx]->ui_flags & PF_PUI_DISABLED) != 0;
	if (disabledNow == !on) return PF_Err_NONE;		// already where we want it

	if (on) params[idx]->ui_flags &= ~PF_PUI_DISABLED;
	else    params[idx]->ui_flags |=  PF_PUI_DISABLED;

	ERR(suites.ParamUtilsSuite3()->PF_UpdateParamUI(in_data->effect_ref, idx, params[idx]));
	return err;
}

// Show or hide a param outright. Same shape as BTS_SetEnabled - and the same
// early-out, because PF_UpdateParamUI on an unchanged param still costs a
// round trip and UpdateParamsUI runs on every param tweak.
static PF_Err
BTS_SetVisible (PF_InData *in_data, PF_ParamDef *params[], A_long idx, A_Boolean on)
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);

	A_Boolean hiddenNow = (params[idx]->ui_flags & PF_PUI_INVISIBLE) != 0;
	if (hiddenNow == !on) return PF_Err_NONE;

	if (on) params[idx]->ui_flags &= ~PF_PUI_INVISIBLE;
	else    params[idx]->ui_flags |=  PF_PUI_INVISIBLE;

	ERR(suites.ParamUtilsSuite3()->PF_UpdateParamUI(in_data->effect_ref, idx, params[idx]));
	return err;
}

static PF_Err
UpdateParamsUI (PF_InData *in_data, PF_OutData *out_data, PF_ParamDef *params[])
{
	PF_Err err = PF_Err_NONE;

	// Main thread: catch up with any Preferences > Labels edit.
	BTS_FetchLabelColors(in_data);

	// Show Overlays off passes the clean composite through untouched, so every
	// overlay control below is inert. This master outranks all the others.
	A_Boolean draws   = params[BTS_SHOW_OVERLAYS]->u.bd.value;
	A_Boolean window  = draws && (params[BTS_VISIBILITY]->u.pd.value == BTS_VIS_WINDOW);
	A_Boolean markers = draws && params[BTS_SHOW_MARKERS]->u.bd.value;
	A_Boolean axes    = draws && params[BTS_SHOW_AXES]->u.bd.value;
	A_Boolean paths   = draws && params[BTS_SHOW_PATHS]->u.bd.value;
	A_Boolean boxes   = draws && params[BTS_SHOW_BOXES]->u.bd.value;
	A_Boolean handles = draws && params[BTS_SHOW_HANDLES]->u.bd.value;
	A_long    colSrc  = params[BTS_COLOR_SOURCE]->u.pd.value;
	// Each color source overrides the pickers belonging to the other two, so a
	// picker is live only when its own source is selected AND its gizmo is on.
	A_Boolean perGizmo = draws && (colSrc == BTS_COLSRC_GIZMO);
	A_Boolean byType   = draws && (colSrc == BTS_COLSRC_TYPE);

	// The transform warning appears only when it applies. UpdateParamsUI is the
	// main thread, which is where AEGP reads belong; PreRender does the same
	// check independently for the render itself.
	ERR(BTS_SetVisible(in_data, params, BTS_WARN, draws && !BTS_XformIsDefault(in_data)));

	ERR(BTS_SetEnabled(in_data, params, BTS_OVERLAYS_ONLY, draws));
	ERR(BTS_SetEnabled(in_data, params, BTS_COLOR_SOURCE,  draws));
	ERR(BTS_SetEnabled(in_data, params, BTS_OPACITY,       draws));

	// Overlay Timing: the window bounds only exist in Window Around CTI mode.
	ERR(BTS_SetEnabled(in_data, params, BTS_VISIBILITY, draws));
	ERR(BTS_SetEnabled(in_data, params, BTS_WIN_START,  window));
	ERR(BTS_SetEnabled(in_data, params, BTS_WIN_LEN,    window));

	// Layer Markers.
	ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_MARKERS, draws));
	ERR(BTS_SetEnabled(in_data, params, BTS_MARKER_STYLE, markers));
	ERR(BTS_SetEnabled(in_data, params, BTS_MARKER_COLOR, markers && perGizmo));
	ERR(BTS_SetEnabled(in_data, params, BTS_MARKER_SIZE,  markers));

	// 3D axes are a SIBLING of markers, not a child - they live in the same
	// topic but SmartRender draws them on their own toggle.
	ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_AXES,  draws));
	ERR(BTS_SetEnabled(in_data, params, BTS_AXIS_LEN,   axes));
	ERR(BTS_SetEnabled(in_data, params, BTS_AXIS_WIDTH, axes));
	ERR(BTS_SetEnabled(in_data, params, BTS_AXIS_HEAD,  axes));

	// Motion Paths. Show Paths IS a true master: SmartRender draws keyframe
	// dots, tangents and frame dots inside its branch, so with it off none of
	// those three can put a pixel down.
	ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_PATHS,     draws));
	ERR(BTS_SetEnabled(in_data, params, BTS_PATH_COLOR,     paths && perGizmo));
	ERR(BTS_SetEnabled(in_data, params, BTS_PATH_THICK,     paths));
	ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_KFDOTS,    paths));
	ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_TANGENTS,  paths));
	ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_FRAMEDOTS, paths));

	// Bounding Boxes.
	ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_BOXES, draws));
	ERR(BTS_SetEnabled(in_data, params, BTS_BOX_COLOR,  boxes && perGizmo));
	ERR(BTS_SetEnabled(in_data, params, BTS_BOX_THICK,  boxes));

	// Handles are NOT a child of Show Boxes. They read the same gathered corner
	// data but SmartRender draws them from their own top-level branch, so they
	// still work with the box outline switched off.
	ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_HANDLES, draws));
	ERR(BTS_SetEnabled(in_data, params, BTS_HANDLE_COLOR, handles && perGizmo));
	ERR(BTS_SetEnabled(in_data, params, BTS_HANDLE_SIZE,  handles));

	// Layer Types: the per-kind SHOW checkboxes are gather filters and stay
	// live whenever we draw; only the swatches next to them depend on
	// Color By Type.
	for (A_long k = 0; k < BTS_LT_COUNT; ++k) {
		ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_CAMERAS + 2 * k,     draws));
		ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_CAMERAS + 2 * k + 1, byType));
	}
	ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_GUIDES, draws));
	ERR(BTS_SetEnabled(in_data, params, BTS_SHOW_LOCKED, draws));

	return err;
}

static PF_Err
UserChangedParam (PF_InData *in_data, PF_OutData *out_data, PF_ParamDef *params[],
                  const PF_UserChangedParamExtra *which)
{
	if (which->param_index == BTS_DUMP)
		return DumpDiagnostics(in_data, out_data, params);
	return PF_Err_NONE;
}

/* =========================================================================
   Anti-aliased rasterization via a COVERAGE buffer.

   Each primitive writes sub-pixel coverage (distance-to-edge, 1px falloff) into
   a float buffer, accumulated with MAX so overlapping strokes never exceed 1.
   One BTS_Composite pass then alpha-overs the whole group's color * opacity in a
   single visit per pixel — smooth edges AND no double-blend at opacity < 1.
   All coords are BUFFER space (comp px already scaled by downsample), fractional
   for sub-pixel placement.
   ========================================================================= */

// Alpha-OVER a straight color at coverage `a` onto the premultiplied output.
/* Alpha-over one pixel at any of AE's three depths. Gizmo colors are STRAIGHT
   0..1; AE's buffers are PREMULTIPLIED, so the source contributes color*a and
   the destination keeps (1-a). Over a fully transparent destination that leaves
   exactly color*a with alpha=a — which is what lets the overlays composite onto
   an empty background instead of needing a solid behind them. */
static void
BTS_PlotAny (PF_EffectWorld *w, A_long depth, A_long x, A_long y,
             float r, float g, float b, float a)
{
	if (a <= 0.0f || x < 0 || y < 0 || x >= w->width || y >= w->height) return;
	if (a > 1.0f) a = 1.0f;
	float ia = 1.0f - a;

	if (depth == 32) {
		PF_PixelFloat *px = &(reinterpret_cast<PF_PixelFloat*>((char*)w->data + (size_t)y * w->rowbytes))[x];
		px->red   = r * a + px->red   * ia;
		px->green = g * a + px->green * ia;
		px->blue  = b * a + px->blue  * ia;
		px->alpha = a     + px->alpha * ia;
	} else if (depth == 16) {
		// CLAMP, do not truncate: a cast of anything over the channel max wraps
		// to a wildly wrong value instead of saturating. Only reachable with an
		// out-of-range color, but that is exactly the failure that is hard to
		// read back from the picture.
		const float M = (float)PF_MAX_CHAN16;
		PF_Pixel16 *px = &(reinterpret_cast<PF_Pixel16*>((char*)w->data + (size_t)y * w->rowbytes))[x];
		px->red   = (A_u_short)(BTS_clamp01(r) * a * M + px->red   * ia + 0.5f);
		px->green = (A_u_short)(BTS_clamp01(g) * a * M + px->green * ia + 0.5f);
		px->blue  = (A_u_short)(BTS_clamp01(b) * a * M + px->blue  * ia + 0.5f);
		px->alpha = (A_u_short)(a * M     + px->alpha * ia + 0.5f);
	} else {
		const float M = (float)PF_MAX_CHAN8;
		PF_Pixel8 *px = &(reinterpret_cast<PF_Pixel8*>((char*)w->data + (size_t)y * w->rowbytes))[x];
		px->red   = (A_u_char)(BTS_clamp01(r) * a * M + px->red   * ia + 0.5f);
		px->green = (A_u_char)(BTS_clamp01(g) * a * M + px->green * ia + 0.5f);
		px->blue  = (A_u_char)(BTS_clamp01(b) * a * M + px->blue  * ia + 0.5f);
		px->alpha = (A_u_char)(a * M     + px->alpha * ia + 0.5f);
	}
}


// Dirty rectangle of touched coverage pixels (so composite/clear stay cheap).
typedef struct { A_long x0, y0, x1, y1; A_Boolean any; } BTSBox;
// Union of two rects; an empty rect contributes nothing.
static void
BTS_RectUnion (PF_LRect *dst, const PF_LRect *add)
{
	if (add->right <= add->left || add->bottom <= add->top) return;
	if (dst->right <= dst->left || dst->bottom <= dst->top) { *dst = *add; return; }
	if (add->left   < dst->left)   dst->left   = add->left;
	if (add->top    < dst->top)    dst->top    = add->top;
	if (add->right  > dst->right)  dst->right  = add->right;
	if (add->bottom > dst->bottom) dst->bottom = add->bottom;
}

static void BTS_BoxReset (BTSBox *b) { b->any = FALSE; }
static void BTS_BoxAdd (BTSBox *b, A_long x, A_long y) {
	if (!b->any) { b->x0 = b->x1 = x; b->y0 = b->y1 = y; b->any = TRUE; }
	else { if (x < b->x0) b->x0 = x; if (x > b->x1) b->x1 = x;
	       if (y < b->y0) b->y0 = y; if (y > b->y1) b->y1 = y; }
}

// Coverage buffer + dims + dirty box, threaded through the primitives.
typedef struct { float *cov; A_long CW, CH; BTSBox bb; } BTSCov;

static void
BTS_Cov (BTSCov *c, A_long x, A_long y, float v)
{
	if (v <= 0.0f || x < 0 || y < 0 || x >= c->CW || y >= c->CH) return;
	float *p = &c->cov[(size_t)y * c->CW + x];
	if (v > *p) { *p = v; BTS_BoxAdd(&c->bb, x, y); }
}

static PF_FpLong
BTS_DistSeg (PF_FpLong px, PF_FpLong py, PF_FpLong ax, PF_FpLong ay, PF_FpLong bx, PF_FpLong by)
{
	PF_FpLong dx = bx - ax, dy = by - ay, len2 = dx*dx + dy*dy;
	PF_FpLong t = (len2 < 1e-9) ? 0.0 : ((px-ax)*dx + (py-ay)*dy) / len2;
	if (t < 0) t = 0; else if (t > 1) t = 1;
	PF_FpLong qx = ax + t*dx, qy = ay + t*dy;
	return sqrt((px-qx)*(px-qx) + (py-qy)*(py-qy));
}

static float
BTS_Clamp01 (PF_FpLong v) { return (float)(v < 0 ? 0 : (v > 1 ? 1 : v)); }

// Finite and within a sane pixel range — guards against NaN/Inf/huge projected
// coords blowing up loop bounds (A_long overflow -> crash).
static A_Boolean
BTS_Ok (PF_FpLong v) { return (v == v) && v < 1e7 && v > -1e7; }

// Round-capped line of half-thickness `half`; coverage = half + 0.5 - dist.
static void
BTS_CovLine (BTSCov *c, PF_FpLong x0, PF_FpLong y0, PF_FpLong x1, PF_FpLong y1, PF_FpLong half)
{
	if (!BTS_Ok(x0) || !BTS_Ok(y0) || !BTS_Ok(x1) || !BTS_Ok(y1)) return;
	PF_FpLong m = half + 1.0;
	A_long minx = (A_long)floor((x0 < x1 ? x0 : x1) - m), maxx = (A_long)ceil((x0 > x1 ? x0 : x1) + m);
	A_long miny = (A_long)floor((y0 < y1 ? y0 : y1) - m), maxy = (A_long)ceil((y0 > y1 ? y0 : y1) + m);
	if (minx < 0) minx = 0;  if (miny < 0) miny = 0;
	if (maxx > c->CW - 1) maxx = c->CW - 1;  if (maxy > c->CH - 1) maxy = c->CH - 1;
	for (A_long y = miny; y <= maxy; ++y)
		for (A_long x = minx; x <= maxx; ++x) {
			PF_FpLong d = BTS_DistSeg((PF_FpLong)x, (PF_FpLong)y, x0, y0, x1, y1);
			BTS_Cov(c, x, y, BTS_Clamp01(half + 0.5 - d));
		}
}

// Filled disk radius `rad`; coverage = rad + 0.5 - dist.
static void
BTS_CovDot (BTSCov *c, PF_FpLong cx, PF_FpLong cy, PF_FpLong rad)
{
	if (!BTS_Ok(cx) || !BTS_Ok(cy)) return;
	A_long e = (A_long)(rad + 1.5);
	A_long cxi = (A_long)floor(cx + 0.5), cyi = (A_long)floor(cy + 0.5);
	A_long y0 = cyi - e < 0 ? 0 : cyi - e, y1 = cyi + e > c->CH - 1 ? c->CH - 1 : cyi + e;
	A_long x0 = cxi - e < 0 ? 0 : cxi - e, x1 = cxi + e > c->CW - 1 ? c->CW - 1 : cxi + e;
	for (A_long y = y0; y <= y1; ++y)
		for (A_long x = x0; x <= x1; ++x) {
			PF_FpLong d = sqrt(((PF_FpLong)x - cx)*((PF_FpLong)x - cx) + ((PF_FpLong)y - cy)*((PF_FpLong)y - cy));
			BTS_Cov(c, x, y, BTS_Clamp01(rad + 0.5 - d));
		}
}

// Axis-aligned filled square, half-side `half`, centered at (cx,cy). Screen-
// aligned (never rotated). AA via separable edge coverage.
static void
BTS_CovSquare (BTSCov *c, PF_FpLong cx, PF_FpLong cy, PF_FpLong half)
{
	if (!BTS_Ok(cx) || !BTS_Ok(cy)) return;
	A_long e = (A_long)(half + 1.5);
	A_long cxi = (A_long)floor(cx + 0.5), cyi = (A_long)floor(cy + 0.5);
	A_long yy0 = cyi - e < 0 ? 0 : cyi - e, yy1 = cyi + e > c->CH - 1 ? c->CH - 1 : cyi + e;
	A_long xx0 = cxi - e < 0 ? 0 : cxi - e, xx1 = cxi + e > c->CW - 1 ? c->CW - 1 : cxi + e;
	for (A_long y = yy0; y <= yy1; ++y) {
		PF_FpLong dyv = (PF_FpLong)y - cy; if (dyv < 0) dyv = -dyv;
		float covY = BTS_Clamp01(half + 0.5 - dyv);
		for (A_long x = xx0; x <= xx1; ++x) {
			PF_FpLong dxv = (PF_FpLong)x - cx; if (dxv < 0) dxv = -dxv;
			BTS_Cov(c, x, y, covY * BTS_Clamp01(half + 0.5 - dxv));
		}
	}
}

// Stroked ring radius R, half-thickness ht; coverage = ht + 0.5 - |dist - R|.
static void
BTS_CovRing (BTSCov *c, PF_FpLong cx, PF_FpLong cy, PF_FpLong R, PF_FpLong ht)
{
	if (!BTS_Ok(cx) || !BTS_Ok(cy)) return;
	A_long e = (A_long)(R + ht + 1.5);
	A_long cxi = (A_long)floor(cx + 0.5), cyi = (A_long)floor(cy + 0.5);
	A_long y0 = cyi - e < 0 ? 0 : cyi - e, y1 = cyi + e > c->CH - 1 ? c->CH - 1 : cyi + e;
	A_long x0 = cxi - e < 0 ? 0 : cxi - e, x1 = cxi + e > c->CW - 1 ? c->CW - 1 : cxi + e;
	for (A_long y = y0; y <= y1; ++y)
		for (A_long x = x0; x <= x1; ++x) {
			PF_FpLong d = sqrt(((PF_FpLong)x - cx)*((PF_FpLong)x - cx) + ((PF_FpLong)y - cy)*((PF_FpLong)y - cy));
			PF_FpLong dd = d - R; if (dd < 0) dd = -dd;
			BTS_Cov(c, x, y, BTS_Clamp01(ht + 0.5 - dd));
		}
}

// Filled triangle (flat, AA'd) through 3 buffer-space verts. Coverage uses a
// signed-distance to the three edges: inside distance is positive, 1px falloff.
static void
BTS_CovTriangle (BTSCov *c, PF_FpLong ax, PF_FpLong ay, PF_FpLong bx, PF_FpLong by,
                 PF_FpLong cx, PF_FpLong cy)
{
	if (!BTS_Ok(ax) || !BTS_Ok(ay) || !BTS_Ok(bx) || !BTS_Ok(by) || !BTS_Ok(cx) || !BTS_Ok(cy)) return;
	A_long minx = (A_long)floor((ax < bx ? (ax < cx ? ax : cx) : (bx < cx ? bx : cx)) - 1);
	A_long maxx = (A_long)ceil ((ax > bx ? (ax > cx ? ax : cx) : (bx > cx ? bx : cx)) + 1);
	A_long miny = (A_long)floor((ay < by ? (ay < cy ? ay : cy) : (by < cy ? by : cy)) - 1);
	A_long maxy = (A_long)ceil ((ay > by ? (ay > cy ? ay : cy) : (by > cy ? by : cy)) + 1);
	if (minx < 0) minx = 0;  if (miny < 0) miny = 0;
	if (maxx > c->CW - 1) maxx = c->CW - 1;  if (maxy > c->CH - 1) maxy = c->CH - 1;

	// Signed area; orient so "inside" edge functions are >= 0.
	PF_FpLong area = (bx - ax)*(cy - ay) - (by - ay)*(cx - ax);
	PF_FpLong s = (area < 0) ? -1.0 : 1.0;

	for (A_long y = miny; y <= maxy; ++y)
		for (A_long x = minx; x <= maxx; ++x) {
			PF_FpLong px = (PF_FpLong)x, py = (PF_FpLong)y;
			// Perp distance to each directed edge (normalized), * orientation.
			PF_FpLong e0 = s * ((bx-ax)*(py-ay) - (by-ay)*(px-ax));
			PF_FpLong e1 = s * ((cx-bx)*(py-by) - (cy-by)*(px-bx));
			PF_FpLong e2 = s * ((ax-cx)*(py-cy) - (ay-cy)*(px-cx));
			PF_FpLong l0 = sqrt((bx-ax)*(bx-ax) + (by-ay)*(by-ay));
			PF_FpLong l1 = sqrt((cx-bx)*(cx-bx) + (cy-by)*(cy-by));
			PF_FpLong l2 = sqrt((ax-cx)*(ax-cx) + (ay-cy)*(ay-cy));
			PF_FpLong d0 = l0 > 1e-9 ? e0 / l0 : e0;
			PF_FpLong d1 = l1 > 1e-9 ? e1 / l1 : e1;
			PF_FpLong d2 = l2 > 1e-9 ? e2 / l2 : e2;
			PF_FpLong dmin = d0 < d1 ? (d0 < d2 ? d0 : d2) : (d1 < d2 ? d1 : d2);
			BTS_Cov(c, x, y, BTS_Clamp01(dmin + 0.5));	// inside(+) -> 1, edge -> soft
		}
}

// Crosshair: two arms + a small open box. lineHalf is the stroke half-width.
static void
BTS_CovCrosshair (BTSCov *c, PF_FpLong cx, PF_FpLong cy,
                  PF_FpLong armX, PF_FpLong armY, PF_FpLong boxX, PF_FpLong boxY, PF_FpLong lineHalf)
{
	BTS_CovLine(c, cx - armX, cy, cx + armX, cy, lineHalf);
	BTS_CovLine(c, cx, cy - armY, cx, cy + armY, lineHalf);
	BTS_CovLine(c, cx - boxX, cy - boxY, cx + boxX, cy - boxY, lineHalf);
	BTS_CovLine(c, cx - boxX, cy + boxY, cx + boxX, cy + boxY, lineHalf);
	BTS_CovLine(c, cx - boxX, cy - boxY, cx - boxX, cy + boxY, lineHalf);
	BTS_CovLine(c, cx + boxX, cy - boxY, cx + boxX, cy + boxY, lineHalf);
}

// AE-style anchor: hollow ring + center dot + 4 outward ticks. E = overall
// extent (tick tip = E), matching Crosshair/Dot extent. Ratios from AE's anchor
// icon (ae-crosshair.svg: ring/tip = 53.9/81.4).
static void
BTS_CovAnchor (BTSCov *c, PF_FpLong cx, PF_FpLong cy, PF_FpLong E)
{
	if (E < 4) E = 4;
	PF_FpLong R = E * 0.66;
	PF_FpLong ringHalf = E * 0.045; if (ringHalf < 0.6) ringHalf = 0.6;
	PF_FpLong centerR = E * 0.12;   if (centerR < 1.0) centerR = 1.0;
	PF_FpLong tickHalf = ringHalf;
	PF_FpLong tin = R + ringHalf + 1.0, tout = E;

	BTS_CovRing(c, cx, cy, R, ringHalf);
	BTS_CovDot(c, cx, cy, centerR);
	BTS_CovLine(c, cx, cy - tin, cx, cy - tout, tickHalf);
	BTS_CovLine(c, cx, cy + tin, cx, cy + tout, tickHalf);
	BTS_CovLine(c, cx - tin, cy, cx - tout, cy, tickHalf);
	BTS_CovLine(c, cx + tin, cy, cx + tout, cy, tickHalf);
}

// Alpha-over the accumulated coverage (color * opacity) into the output, one
// visit per touched pixel, then zero the touched region so the buffer is reusable.
static void
BTS_Composite (PF_EffectWorld *w, A_long depth, BTSCov *c, float r, float g, float b, float op)
{
	if (!c->bb.any) return;
	A_long x0 = c->bb.x0 < 0 ? 0 : c->bb.x0, y0 = c->bb.y0 < 0 ? 0 : c->bb.y0;
	A_long x1 = c->bb.x1 >= c->CW ? c->CW - 1 : c->bb.x1;
	A_long y1 = c->bb.y1 >= c->CH ? c->CH - 1 : c->bb.y1;
	for (A_long y = y0; y <= y1; ++y) {
		float *row = &c->cov[(size_t)y * c->CW];
		for (A_long x = x0; x <= x1; ++x) {
			float v = row[x];
			if (v > 0.0f) { BTS_PlotAny(w, depth, x, y, r, g, b, v * op); row[x] = 0.0f; }
		}
	}
	BTS_BoxReset(&c->bb);
}

// Comp-space size (px) -> buffer px (fractional), floored to a small minimum.
static PF_FpLong
BTS_ScaleF (A_long compPx, PF_FpLong ds)
{
	PF_FpLong s = (PF_FpLong)compPx * ds;
	return s < 0.5 ? 0.5 : s;
}

/* =========================================================================
   SmartFX
   ========================================================================= */

static PF_Err
PreRender (PF_InData *in_data, PF_OutData *out_data, PF_PreRenderExtra *extra)
{
	PF_Err				err = PF_Err_NONE;
	PF_RenderRequest	req = extra->input->output_request;
	PF_CheckoutResult	in_result;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);

	PF_Handle infoH = suites.HandleSuite1()->host_new_handle(sizeof(BTSInfo));
	if (!infoH) return PF_Err_OUT_OF_MEMORY;

	BTSInfo *infoP = reinterpret_cast<BTSInfo*>(suites.HandleSuite1()->host_lock_handle(infoH));
	if (!infoP) { suites.HandleSuite1()->host_dispose_handle(infoH); return PF_Err_OUT_OF_MEMORY; }

	extra->output->pre_render_data = infoH;
	AEFX_CLR_STRUCT(*infoP);

	#define BTS_CO(IDX, PDEF) PF_CHECKOUT_PARAM(in_data, (IDX), in_data->current_time, \
							in_data->time_step, in_data->time_scale, &(PDEF))
	PF_ParamDef view_p, so_p, oo_p, op_p, sm_p, mst_p, mc_p, ms_p, sp_p, pc_p, pt_p, kd_p, tg_p, fd_p, sb_p, bc_p, bt_p, sh_p, hc_p, hs_p, sax_p, axl_p, axw_p, ahd_p, vis_p, ws_p, wl_p;
	AEFX_CLR_STRUCT(view_p); AEFX_CLR_STRUCT(so_p); AEFX_CLR_STRUCT(oo_p); AEFX_CLR_STRUCT(op_p); AEFX_CLR_STRUCT(sm_p);
	AEFX_CLR_STRUCT(mst_p);  AEFX_CLR_STRUCT(mc_p); AEFX_CLR_STRUCT(ms_p);
	AEFX_CLR_STRUCT(sp_p);   AEFX_CLR_STRUCT(pc_p); AEFX_CLR_STRUCT(pt_p);
	AEFX_CLR_STRUCT(kd_p);   AEFX_CLR_STRUCT(tg_p); AEFX_CLR_STRUCT(fd_p);
	AEFX_CLR_STRUCT(sb_p);   AEFX_CLR_STRUCT(bc_p); AEFX_CLR_STRUCT(bt_p);
	AEFX_CLR_STRUCT(sh_p);   AEFX_CLR_STRUCT(hc_p); AEFX_CLR_STRUCT(hs_p);
	AEFX_CLR_STRUCT(sax_p);  AEFX_CLR_STRUCT(axl_p); AEFX_CLR_STRUCT(axw_p); AEFX_CLR_STRUCT(ahd_p);
	AEFX_CLR_STRUCT(vis_p);  AEFX_CLR_STRUCT(ws_p); AEFX_CLR_STRUCT(wl_p);
	ERR(BTS_CO(BTS_VIEW, view_p));
	ERR(BTS_CO(BTS_SHOW_OVERLAYS, so_p));
	ERR(BTS_CO(BTS_OVERLAYS_ONLY, oo_p));
	ERR(BTS_CO(BTS_OPACITY, op_p));
	ERR(BTS_CO(BTS_VISIBILITY, vis_p));
	ERR(BTS_CO(BTS_WIN_START, ws_p));
	ERR(BTS_CO(BTS_WIN_LEN, wl_p));
	ERR(BTS_CO(BTS_SHOW_MARKERS, sm_p));
	ERR(BTS_CO(BTS_MARKER_STYLE, mst_p));
	ERR(BTS_CO(BTS_MARKER_COLOR, mc_p));
	ERR(BTS_CO(BTS_MARKER_SIZE, ms_p));
	ERR(BTS_CO(BTS_SHOW_AXES, sax_p));
	ERR(BTS_CO(BTS_AXIS_LEN, axl_p));
	ERR(BTS_CO(BTS_AXIS_WIDTH, axw_p));
	ERR(BTS_CO(BTS_AXIS_HEAD, ahd_p));
	ERR(BTS_CO(BTS_SHOW_PATHS, sp_p));
	ERR(BTS_CO(BTS_PATH_COLOR, pc_p));
	ERR(BTS_CO(BTS_PATH_THICK, pt_p));
	ERR(BTS_CO(BTS_SHOW_KFDOTS, kd_p));
	ERR(BTS_CO(BTS_SHOW_TANGENTS, tg_p));
	ERR(BTS_CO(BTS_SHOW_FRAMEDOTS, fd_p));
	ERR(BTS_CO(BTS_SHOW_BOXES, sb_p));
	ERR(BTS_CO(BTS_BOX_COLOR, bc_p));
	ERR(BTS_CO(BTS_BOX_THICK, bt_p));
	ERR(BTS_CO(BTS_SHOW_HANDLES, sh_p));
	ERR(BTS_CO(BTS_HANDLE_COLOR, hc_p));
	ERR(BTS_CO(BTS_HANDLE_SIZE, hs_p));

	// Layer Types: Color By Type + a show/color pair per kind (indexed by
	// BTS_LT_*, so the param pairs must stay in kind order), + the two attribute
	// filters. Checked out into arrays so the gather loop can index by kind.
	PF_ParamDef cbt_p, kshow_p[BTS_LT_COUNT], kcol_p[BTS_LT_COUNT], gd_p, lk_p;
	AEFX_CLR_STRUCT(cbt_p); AEFX_CLR_STRUCT(gd_p); AEFX_CLR_STRUCT(lk_p);
	for (A_long k = 0; k < BTS_LT_COUNT; ++k) { AEFX_CLR_STRUCT(kshow_p[k]); AEFX_CLR_STRUCT(kcol_p[k]); }
	ERR(BTS_CO(BTS_COLOR_SOURCE, cbt_p));
	for (A_long k = 0; k < BTS_LT_COUNT; ++k) {
		ERR(BTS_CO(BTS_SHOW_CAMERAS + 2 * k,     kshow_p[k]));
		ERR(BTS_CO(BTS_SHOW_CAMERAS + 2 * k + 1, kcol_p[k]));
	}
	ERR(BTS_CO(BTS_SHOW_GUIDES, gd_p));
	ERR(BTS_CO(BTS_SHOW_LOCKED, lk_p));
	#undef BTS_CO

	req.preserve_rgb_of_zero_alpha = TRUE;
	req.field = PF_Field_FRAME;
	ERR(extra->cb->checkout_layer(in_data->effect_ref, BTS_INPUT, BTS_INPUT,
								  &req, in_data->current_time, in_data->time_step,
								  in_data->time_scale, &in_result));

	if (!err) {
		infoP->viewMode    = view_p.u.pd.value;
		// Derive the old three-way mode from the two checkboxes. Overlays Only
		// is meaningless with overlays off, so Show Overlays wins outright -
		// which is also why the UI greys it out in that state.
		infoP->outputMode  = !so_p.u.bd.value ? BTS_OUT_INPUT
							 : (oo_p.u.bd.value ? BTS_OUT_OVERLAYS : BTS_OUT_COMPOSITE);
		infoP->downsampleX = (PF_FpLong)in_data->downsample_x.num / (PF_FpLong)in_data->downsample_x.den;
		infoP->downsampleY = (PF_FpLong)in_data->downsample_y.num / (PF_FpLong)in_data->downsample_y.den;
		infoP->width       = in_result.max_result_rect.right  - in_result.max_result_rect.left;
		infoP->height      = in_result.max_result_rect.bottom - in_result.max_result_rect.top;
		infoP->inLeft      = in_result.result_rect.left;
		infoP->inTop       = in_result.result_rect.top;

		infoP->opacity     = op_p.u.fs_d.value / 100.0;
		infoP->showMarkers = sm_p.u.bd.value;
		infoP->markerStyle = mst_p.u.pd.value;
		infoP->showAxes    = sax_p.u.bd.value;
		infoP->axisLen     = (A_long)(axl_p.u.fs_d.value + 0.5);
		infoP->axisWidth   = (A_long)(axw_p.u.fs_d.value + 0.5);
		infoP->axisHead    = ahd_p.u.pd.value;
		infoP->showPaths   = sp_p.u.bd.value;
		infoP->showKfDots  = kd_p.u.bd.value;
		infoP->showTangents= tg_p.u.bd.value;
		infoP->showFrameDots = fd_p.u.bd.value;
		infoP->mColR = BTS_c(mc_p.u.cd.value.red);   infoP->mColG = BTS_c(mc_p.u.cd.value.green); infoP->mColB = BTS_c(mc_p.u.cd.value.blue);
		infoP->pColR = BTS_c(pc_p.u.cd.value.red);   infoP->pColG = BTS_c(pc_p.u.cd.value.green); infoP->pColB = BTS_c(pc_p.u.cd.value.blue);
		infoP->markerSize  = (A_long)(ms_p.u.fs_d.value + 0.5);
		infoP->pathThick   = (A_long)(pt_p.u.fs_d.value + 0.5);
		infoP->showBoxes   = sb_p.u.bd.value;
		infoP->bColR = BTS_c(bc_p.u.cd.value.red); infoP->bColG = BTS_c(bc_p.u.cd.value.green); infoP->bColB = BTS_c(bc_p.u.cd.value.blue);
		infoP->boxThick    = (A_long)(bt_p.u.fs_d.value + 0.5);
		infoP->showHandles = sh_p.u.bd.value;
		infoP->hColR = BTS_c(hc_p.u.cd.value.red); infoP->hColG = BTS_c(hc_p.u.cd.value.green); infoP->hColB = BTS_c(hc_p.u.cd.value.blue);
		infoP->handleSize  = (A_long)(hs_p.u.fs_d.value + 0.5);
		infoP->colorSource = cbt_p.u.pd.value;
		if (!S_labelColValid) BTS_FetchLabelColors(in_data);
		memcpy(infoP->labelCol, S_labelCol, sizeof(S_labelCol));
		for (A_long k = 0; k < BTS_LT_COUNT; ++k) {
			infoP->showType[k] = kshow_p[k].u.bd.value;
			infoP->tCol[k][0] = BTS_c(kcol_p[k].u.cd.value.red);
			infoP->tCol[k][1] = BTS_c(kcol_p[k].u.cd.value.green);
			infoP->tCol[k][2] = BTS_c(kcol_p[k].u.cd.value.blue);
		}

		// Union the input's rects with what was REQUESTED. Over a transparent
		// composite (no solid behind us) the input's result rect can come back
		// empty, AE then allocates no canvas, and the overlays have nowhere to
		// land. Our gizmos are generated rather than derived from input pixels,
		// so filling the whole requested area is always legitimate.
		PF_LRect rr = in_result.result_rect, mr = in_result.max_result_rect;
		BTS_RectUnion(&rr, &req.rect);
		BTS_RectUnion(&mr, &req.rect);
		extra->output->result_rect     = rr;
		extra->output->max_result_rect = mr;
	}

	// --- read layer Positions via AEGP, from PreRender ---
	// Time Range decides WHAT gets gathered:
	//   Whole Composition : every layer, paths span the work area (duration-agnostic).
	//   Window Around CTI : only layers visible at the CTI (eyeball on + in/out spans
	//                       the time); each path is clamped to [CTI-before, CTI+after]
	//                       intersected with that layer's own in/out.
	A_Boolean windowMode = (vis_p.u.pd.value == BTS_VIS_WINDOW);
	PF_FpLong beforeSec = ws_p.u.fs_d.value, afterSec = wl_p.u.fs_d.value;
	A_Boolean showGuides = gd_p.u.bd.value, showLocked = lk_p.u.bd.value;

	// Nothing we could gather can reach a pixel when the effect is passing the
	// clean input through, or when the master opacity is zero - SmartRender
	// skips the whole draw in both cases. So skip the AEGP scene walk too.
	// Transform guard. Checked BEFORE the early-out below so the warning still
	// renders at zero opacity - the X is a diagnostic, not a gizmo, and having
	// it silently vanish with the opacity slider would hide the very failure it
	// exists to report. Only Show Overlays off suppresses it, because that mode
	// promises the untouched composite.
	infoP->xformBad = (infoP->outputMode != BTS_OUT_INPUT) && !BTS_XformIsDefault(in_data);

	// A broken transform means every gizmo would land in the wrong place, so
	// SmartRender draws the X instead of any of them - no point gathering.
	if (infoP->outputMode == BTS_OUT_INPUT || infoP->opacity <= 0.0 || infoP->xformBad) {
		suites.HandleSuite1()->host_unlock_handle(infoH);
		return err;
	}

	if (!err) {
		// Quiet AEGP errors: optional per-layer reads (keyframes, tangents,
		// bounds) can legitimately fail on odd layer/stream states — swallow
		// them instead of surfacing AE's "no key values" modal.
		AEGP_ErrReportState errState;
		suites.UtilitySuite3()->AEGP_StartQuietErrors(&errState);

		AEGP_CompH compH = NULL;
		A_long myIdx = 0, num = 0;
		PF_Err e = BTS_GetContext(in_data, &compH, &myIdx, &num);
		if (!e && compH && num > 0) {
			// Whole-comp path extent = the work area.
			A_Time waStart, waDur;
			AEFX_CLR_STRUCT(waStart); AEFX_CLR_STRUCT(waDur);
			suites.CompSuite12()->AEGP_GetCompWorkAreaStart(compH, &waStart);
			suites.CompSuite12()->AEGP_GetCompWorkAreaDuration(compH, &waDur);

			A_FpLong fps = 0;
			suites.CompSuite12()->AEGP_GetCompFramerate(compH, &fps);

			PF_FpLong scale = (PF_FpLong)in_data->time_scale;
			PF_FpLong tSec  = (scale > 0) ? (PF_FpLong)in_data->current_time / scale : 0.0;

			// Active camera for projecting 3D-layer points to screen.
			A_Time nowT;  nowT.value = in_data->current_time;  nowT.scale = in_data->time_scale;
			BTSCam cam;
			BTS_BuildCamera(in_data, compH, &nowT, &cam);

			// Show Paths gates the ENTIRE path group in SmartRender - kf dots,
			// tangents and frame dots are all drawn inside its branch - so with
			// it off there is no reason to sample any path at all.
			A_Boolean anyPathGizmo = infoP->showPaths;
			// (3) Sample budget shared by ALL layers this PreRender. Layers gathered
			// later coarsen their grid step rather than being dropped outright.
			A_long budgetLeft = BTS_TOTAL_BUDGET;

			A_long cap = (num < BTS_MAX_MARKERS) ? num : BTS_MAX_MARKERS;
			for (A_long i = 0; i < cap; ++i) {
				if (i == myIdx) continue;	// don't overlay our own adjustment layer

				// Window mode: skip layers not on-screen at the CTI. Also grab the
				// layer's in/out (comp sec) to clamp its path window.
				PF_FpLong inSec = 0, outSec = 0;
				if (windowMode &&
					!BTS_LayerVisibleAt(in_data, compH, i, in_data->current_time, &inSec, &outSec))
					continue;

				AEGP_LayerH layerH = NULL;
				A_Boolean is3D = FALSE;
				suites.LayerSuite9()->AEGP_GetCompLayerByIndex(compH, i, &layerH);
				if (layerH) suites.LayerSuite9()->AEGP_IsLayer3D(layerH, &is3D);

				// Layer-type discriminators: kind decides whether this layer is
				// gathered at all (and, later, its color); guide/locked are
				// separate attribute filters that veto any kind.
				A_Boolean isGuide = FALSE, isLocked = FALSE;
				A_long kind = BTS_LayerKind(in_data, layerH, &isGuide, &isLocked);
				if (!infoP->showType[kind]) continue;
				if (isGuide  && !showGuides) continue;
				if (isLocked && !showLocked) continue;

				PF_FpLong x = 0, y = 0, z = 0;
				// Per-layer failure is skipped (some layer types have no Position).
				if (BTS_ReadLayer(in_data, compH, i, &x, &y, &z, NULL, NULL, 0) == PF_Err_NONE) {
					A_long mslot = infoP->markerCount;
					infoP->mKind[mslot] = kind;
					// AE label slot (0 = No Label). Cheap - just an index; the
					// color it maps to came from the main-thread snapshot.
					AEGP_LabelID lbl = AEGP_Label_NONE;
					infoP->mLabel[mslot] = (layerH &&
						suites.LayerSuite9()->AEGP_GetLayerLabel(layerH, &lbl) == PF_Err_NONE
						&& lbl > 0) ? (A_long)lbl : 0;
					BTS_ProjectPt(&cam, is3D, x, y, z, &infoP->markerX[mslot], &infoP->markerY[mslot]);
					infoP->boxValid[mslot] = BTS_ReadBox(in_data, compH, i, &cam, is3D,
							infoP->boxX[mslot], infoP->boxY[mslot], &infoP->box3D[mslot]);
					infoP->axisValid[mslot] = is3D && BTS_ReadAxes(in_data, compH, i, &cam,
							(PF_FpLong)infoP->axisLen,
							&infoP->axisOX[mslot], &infoP->axisOY[mslot],
							infoP->axisEX[mslot], infoP->axisEY[mslot]);
					infoP->markerCount++;
				}

				// If nothing that depends on the motion path is switched on, the
				// whole path pipeline (window math, keyframe reads, sampling) is
				// dead work for this layer and every other one.
				if (!anyPathGizmo) continue;

				// (4) STATIC POSITION => marker only. One IsStreamTimevarying
				// call (it counts expressions too) settles it before any of the
				// expensive reads below. Every layer that never moves - solids,
				// backgrounds, most adjustment layers - now costs exactly this.
				AEGP_StreamRefH posH = BTS_NewPosStream(in_data, layerH);
				if (!posH) continue;
				if (!BTS_PosIsAnimated(in_data, posH)) {
					suites.StreamSuite6()->AEGP_DisposeStream(posH);
					continue;
				}

				// Choose this layer's motion-path extent (comp sec).
				// ALWAYS clamp to the layer's KEYFRAME SPAN: outside it Position is
				// static, so sampling there burns vertices on a stationary point and
				// coarsens the moving part.
				// Window mode then intersects the CTI window on top, so past the last
				// key the leading edge pins there and the trail erodes frame by frame.
				// Unkeyed + whole-comp falls back to the work area.
				PF_FpLong ws = -1e18, we = 1e18;
				PF_FpLong kfFirst = 0, kfLast = 0;
				A_Boolean haveKf = (BTS_KfRange(suites, posH, &kfFirst, &kfLast) >= 2);
				if (haveKf) { ws = kfFirst; we = kfLast; }

				if (windowMode) {
					PF_FpLong cs = tSec - beforeSec, ce = tSec + afterSec;
					if (cs > ws) ws = cs;				// intersect the CTI window
					if (ce < we) we = ce;
					if (ws < inSec)  ws = inSec;		// and the layer's own span
					if (we > outSec) we = outSec;
				}
				if (!haveKf && !windowMode) {
					ws = (waStart.scale > 0) ? (PF_FpLong)waStart.value / (PF_FpLong)waStart.scale : 0.0;
					we = ws + ((waDur.scale > 0) ? (PF_FpLong)waDur.value / (PF_FpLong)waDur.scale : 0.0);
				}

				// Nothing to draw (marker still shown), or no room left in the budget.
				if (we <= ws || fps <= 0 ||
					infoP->pathCount >= BTS_MAX_PATHS || budgetLeft < 8) {
					suites.StreamSuite6()->AEGP_DisposeStream(posH);
					continue;
				}

				// (1)+(3) Lay the window on the ABSOLUTE 1/BTS_SUBDIV-frame grid and
				// pick a step that scales with its length: short windows keep the
				// sub-frame detail, long ones coarsen instead of burning samples.
				// The second loop lets the shared budget coarsen it further, so the
				// first few layers cannot starve the rest.
				A_FpLong spanFine = (we - ws) * fps * (A_FpLong)BTS_SUBDIV;
				A_long step = 1;
				while (step < 65536 && spanFine / (A_FpLong)step + 1.0 > (A_FpLong)BTS_TARGET_PTS) step <<= 1;
				while (step < 65536 && spanFine / (A_FpLong)step + 1.0 > (A_FpLong)budgetLeft)     step <<= 1;

				// Grid indices are absolute and step-aligned, so the SAME index maps
				// to the SAME time on every frame - that is what makes both AE's
				// stream cache and our own slide-reuse hit.
				A_long kLo = (A_long)ceil (ws * fps * (A_FpLong)BTS_SUBDIV / (A_FpLong)step - 1e-6) * step;
				A_long kHi = (A_long)floor(we * fps * (A_FpLong)BTS_SUBDIV / (A_FpLong)step + 1e-6) * step;
				if (kHi - kLo < step) {
					suites.StreamSuite6()->AEGP_DisposeStream(posH);
					continue;
				}

				// (2) Cached world-space sampling; projection happens after, so an
				// animated camera never invalidates the cache.
				AEGP_LayerIDVal layerID = AEGP_LayerIDVal_NONE;
				suites.LayerSuite9()->AEGP_GetLayerID(layerH, &layerID);
				AEGP_LayerH parentH = BTS_GetParent(suites, layerH);

				// CACHE HOLE: the signature only checksums THIS layer's Position
				// keyframes, so it cannot see the parent being moved or
				// re-animated - and a parented layer's comp-space path depends
				// entirely on that. Sig 0 disables caching, so parented layers
				// re-sample every frame. They keep the frame-grid quantization,
				// which is what lets AE's own stream cache carry most of the
				// cost, so this is a much smaller loss than it sounds.
				PF_FpLong sig = parentH ? 0.0 : BTS_PosSig(suites, posH);
				PF_FpLong sx[BTS_PATH_SAMPLES], sy[BTS_PATH_SAMPLES], sz[BTS_PATH_SAMPLES];
				A_long np = BTS_SampleGrid(in_data, posH, parentH, compH, layerID, sig, kLo, kHi, step,
										   fps, (A_long)in_data->time_scale, sx, sy, sz);
				if (np > 1) {
					A_long slot = infoP->pathCount;
					budgetLeft -= np;
					infoP->pathKind[slot] = kind;
					{	AEGP_LabelID plbl = AEGP_Label_NONE;
						infoP->pathLabel[slot] = (layerH &&
							suites.LayerSuite9()->AEGP_GetLayerLabel(layerH, &plbl) == PF_Err_NONE
							&& plbl > 0) ? (A_long)plbl : 0;  }
					infoP->pathNPts[slot] = np;
					for (A_long s = 0; s < np; ++s)
						BTS_ProjectPt(&cam, is3D, sx[s], sy[s], sz[s],
									  &infoP->pathX[slot][s], &infoP->pathY[slot][s]);

					// Per-frame velocity dots are the whole-frame SUBSET of the
					// samples we already hold (grid index divisible by BTS_SUBDIV) -
					// the old code paid for a second full pass over the stream.
					A_long fstride = (step >= BTS_SUBDIV) ? 1 : (BTS_SUBDIV / step);
					A_long foff = 0;
					if (step < BTS_SUBDIV) {
						A_long r = kLo % BTS_SUBDIV;  if (r < 0) r += BTS_SUBDIV;
						foff = ((BTS_SUBDIV - r) % BTS_SUBDIV) / step;
					}
					A_long fn = 0;
					for (A_long s = foff; s < np && fn < BTS_MAX_FRAMES; s += fstride) {
						infoP->frameX[slot][fn] = infoP->pathX[slot][s];
						infoP->frameY[slot][fn] = infoP->pathY[slot][s];
						fn++;
					}
					infoP->frameN[slot] = fn;

					infoP->kfN[slot] = BTS_ReadKeyframes(suites, posH, parentH, &cam, is3D,
							infoP->kfX[slot], infoP->kfY[slot],
							infoP->kfInX[slot], infoP->kfInY[slot],
							infoP->kfOutX[slot], infoP->kfOutY[slot], BTS_MAX_KF, ws, we);
					infoP->pathCount++;
				}
				suites.StreamSuite6()->AEGP_DisposeStream(posH);
			}
		}
		// AEGP failures are non-fatal for the render — passthrough still works.
		suites.UtilitySuite3()->AEGP_EndQuietErrors(FALSE, &errState);
	}

	suites.HandleSuite1()->host_unlock_handle(infoH);
	return err;
}

static PF_Err
SmartRender (PF_InData *in_data, PF_OutData *out_data, PF_SmartRenderExtra *extra)
{
	PF_Err				err = PF_Err_NONE;
	AEGP_SuiteHandler	suites(in_data->pica_basicP);
	PF_EffectWorld		*inputP = NULL, *outputP = NULL;

	BTSInfo *infoP = reinterpret_cast<BTSInfo*>(
		suites.HandleSuite1()->host_lock_handle(reinterpret_cast<PF_Handle>(extra->input->pre_render_data)));
	if (!infoP) return PF_Err_BAD_CALLBACK_PARAM;

	ERR(extra->cb->checkout_layer_pixels(in_data->effect_ref, BTS_INPUT, &inputP));
	ERR(extra->cb->checkout_output(in_data->effect_ref, &outputP));

	if (!err && inputP && outputP) {
		// Background. Composite/Input pass the layers below straight through at
		// any depth; Only Overlays starts from transparency instead, so the
		// gizmos come out on a clean alpha channel ready to composite
		// elsewhere (zero bytes = transparent black at 8, 16 and 32 bpc).
		if (infoP->outputMode == BTS_OUT_OVERLAYS) {
			for (A_long yy = 0; yy < outputP->height; ++yy)
				memset((char*)outputP->data + (size_t)yy * outputP->rowbytes, 0,
					   (size_t)outputP->rowbytes);
		} else {
			// PLACE the composite below - do not stretch it.
			//
			// copy() maps src_r onto dst_r and SCALES between them; the SDK's
			// own Resizer sample resizes images with this exact call. So
			// whole-world -> whole-world silently stretches whenever the two
			// worlds differ in size, which is precisely what PreRender arranges
			// when there is no full-frame background: the input shrinks to the
			// bounds of whatever is actually visible below, while our output is
			// widened to the full requested rect so the gizmos have a canvas.
			// The result was the composite below blown up to fill the frame.
			A_long dx = infoP->inLeft - in_data->output_origin_x;
			A_long dy = infoP->inTop  - in_data->output_origin_y;

			if (dx == 0 && dy == 0 &&
				inputP->width == outputP->width && inputP->height == outputP->height) {
				// Aligned and same size - the common case, and 1:1 either way.
				ERR(suites.WorldTransformSuite1()->copy(in_data->effect_ref, inputP, outputP, NULL, NULL));
			} else {
				// Anything the input does not cover has to start transparent,
				// otherwise it keeps whatever AE left in the buffer.
				for (A_long yy = 0; yy < outputP->height; ++yy)
					memset((char*)outputP->data + (size_t)yy * outputP->rowbytes, 0,
						   (size_t)outputP->rowbytes);

				// Same WIDTH and HEIGHT as the source, so copy() places instead
				// of scaling. Off-buffer rects are clipped by AE.
				PF_Rect dst;
				dst.left   = dx;
				dst.top    = dy;
				dst.right  = dx + inputP->width;
				dst.bottom = dy + inputP->height;
				ERR(suites.WorldTransformSuite1()->copy(in_data->effect_ref, inputP, outputP, NULL, &dst));
			}
		}

		// Overlay: anti-aliased gizmos via a coverage buffer, composited at the
		// output's own depth (8 / 16 / 32).
		A_long depth = extra->input->bitdepth;
		float a = (float)infoP->opacity;
		BTSCov cbuf; cbuf.cov = NULL;
		PF_FpLong dsx = (PF_FpLong)in_data->downsample_x.num / (PF_FpLong)in_data->downsample_x.den;
		PF_FpLong dsy = (PF_FpLong)in_data->downsample_y.num / (PF_FpLong)in_data->downsample_y.den;
		PF_FpLong ox = (PF_FpLong)in_data->output_origin_x, oy = (PF_FpLong)in_data->output_origin_y;
		#define BTS_MFX(cx) ((PF_FpLong)(cx) * dsx - ox)
		#define BTS_MFY(cy) ((PF_FpLong)(cy) * dsy - oy)
		// Per-LAYER color (By Layer Type / By Label) turns each gizmo group from
		// ONE composite pass into one pass PER LAYER. BTS_Composite zeroes the
		// region it touched, so the coverage buffer is reusable between flushes.
		// A layer with no label falls back to the group color, which is why the
		// group color is passed in here too.
		#define BTS_FLUSH_LAYER(K, L, R, G, B) do {                              \
			if (infoP->colorSource == BTS_COLSRC_TYPE)                           \
				BTS_Composite(outputP, depth, &cbuf,                             \
					infoP->tCol[K][0], infoP->tCol[K][1], infoP->tCol[K][2], a);  \
			else if (infoP->colorSource == BTS_COLSRC_LABEL) {                   \
				A_long lb_ = (L);                                                \
				if (lb_ > 0 && lb_ <= BTS_LABEL_COUNT)                           \
					BTS_Composite(outputP, depth, &cbuf,                         \
						infoP->labelCol[lb_][0], infoP->labelCol[lb_][1],         \
						infoP->labelCol[lb_][2], a);                             \
				else                                                             \
					BTS_Composite(outputP, depth, &cbuf, (R), (G), (B), a);       \
			}                                                                    \
		} while (0)
		// Per Gizmo composites the whole group in one pass instead.
		#define BTS_FLUSH_GROUP(R,G,B) do { if (infoP->colorSource == BTS_COLSRC_GIZMO) BTS_Composite(outputP, depth, &cbuf, (R), (G), (B), a); } while (0)

		// The X is drawn at its own fixed opacity, so it needs the coverage
		// buffer even when the master opacity slider is at zero.
		if (!err && infoP->outputMode != BTS_OUT_INPUT && (a > 0.0f || infoP->xformBad)) {
			cbuf.CW = outputP->width; cbuf.CH = outputP->height; BTS_BoxReset(&cbuf.bb);
			size_t nCov = (size_t)cbuf.CW * (size_t)cbuf.CH;
			cbuf.cov = (float*)malloc(nCov * sizeof(float));
			if (cbuf.cov) memset(cbuf.cov, 0, nCov * sizeof(float));
		}

		if (cbuf.cov && infoP->xformBad) {
			// Host layer is transformed: a big red X plus a frame, covering the
			// whole output. PreRender gathered no geometry in this state, so the
			// gizmo blocks below all iterate zero times and this is the only
			// thing that reaches a pixel.
			//
			// Deliberately NOT scaled by the master opacity - a warning the user
			// can fade out is a warning they will miss. It is loud on purpose:
			// the alternative is a render full of overlays that are quietly,
			// plausibly, in the wrong place.
			PF_FpLong W = (PF_FpLong)outputP->width - 1.0, H = (PF_FpLong)outputP->height - 1.0;
			PF_FpLong half = BTS_ScaleF(8, dsx) * 0.5;
			BTS_CovLine(&cbuf, 0, 0, W, H, half);
			BTS_CovLine(&cbuf, W, 0, 0, H, half);
			BTS_CovLine(&cbuf, 0, 0, W, 0, half);
			BTS_CovLine(&cbuf, W, 0, W, H, half);
			BTS_CovLine(&cbuf, W, H, 0, H, half);
			BTS_CovLine(&cbuf, 0, H, 0, 0, half);
			BTS_Composite(outputP, depth, &cbuf, 1.0f, 0.12f, 0.10f, 0.9f);
		}

		if (cbuf.cov) {
			// 1) motion paths (composited first, so markers sit on top)
			if (infoP->showPaths) {
				PF_FpLong pHalf  = BTS_ScaleF(infoP->pathThick, dsx) * 0.5;
				PF_FpLong tanDot = BTS_ScaleF(3, dsx);
				PF_FpLong kfDot  = BTS_ScaleF(6, dsx);
				PF_FpLong frDot  = BTS_ScaleF(2, dsx);
				for (A_long p = 0; p < infoP->pathCount; ++p) {
					A_long np = infoP->pathNPts[p];
					for (A_long k = 1; k < np; ++k)
						BTS_CovLine(&cbuf,
							BTS_MFX(infoP->pathX[p][k-1]), BTS_MFY(infoP->pathY[p][k-1]),
							BTS_MFX(infoP->pathX[p][k]),   BTS_MFY(infoP->pathY[p][k]), pHalf);

					if (infoP->showFrameDots)
						for (A_long k = 0; k < infoP->frameN[p]; ++k)
							BTS_CovDot(&cbuf, BTS_MFX(infoP->frameX[p][k]), BTS_MFY(infoP->frameY[p][k]), frDot);

					if (infoP->showTangents)
						for (A_long j = 0; j < infoP->kfN[p]; ++j) {
							PF_FpLong kx = BTS_MFX(infoP->kfX[p][j]),    ky = BTS_MFY(infoP->kfY[p][j]);
							PF_FpLong hix = BTS_MFX(infoP->kfInX[p][j]),  hiy = BTS_MFY(infoP->kfInY[p][j]);
							PF_FpLong hox = BTS_MFX(infoP->kfOutX[p][j]), hoy = BTS_MFY(infoP->kfOutY[p][j]);
							BTS_CovLine(&cbuf, kx, ky, hix, hiy, 0.5);
							BTS_CovLine(&cbuf, kx, ky, hox, hoy, 0.5);
							BTS_CovDot(&cbuf, hix, hiy, tanDot);
							BTS_CovDot(&cbuf, hox, hoy, tanDot);
						}

					if (infoP->showKfDots)
						for (A_long j = 0; j < infoP->kfN[p]; ++j)
							BTS_CovDot(&cbuf, BTS_MFX(infoP->kfX[p][j]), BTS_MFY(infoP->kfY[p][j]), kfDot);

					BTS_FLUSH_LAYER(infoP->pathKind[p], infoP->pathLabel[p], infoP->pColR, infoP->pColG, infoP->pColB);
				}
				BTS_FLUSH_GROUP(infoP->pColR, infoP->pColG, infoP->pColB);
			}

			// 2) bounding boxes (composited under markers)
			if (infoP->showBoxes) {
				PF_FpLong bHalf = BTS_ScaleF(infoP->boxThick, dsx) * 0.5;
				for (A_long i = 0; i < infoP->markerCount; ++i) {
					if (!infoP->boxValid[i]) continue;
					PF_FpLong *bx = infoP->boxX[i], *by = infoP->boxY[i];
					if (infoP->box3D[i]) {
						// 8-corner mesh box: 12 edges. Corner order is set in
						// BTS_ReadBox (0-3 back face, 4-7 front face).
						static const A_long e3d[12][2] = {
							{0,1},{1,2},{2,3},{3,0},		// back face
							{4,5},{5,6},{6,7},{7,4},		// front face
							{0,4},{1,5},{2,6},{3,7}			// connecting edges
						};
						for (A_long e = 0; e < 12; ++e)
							BTS_CovLine(&cbuf, BTS_MFX(bx[e3d[e][0]]), BTS_MFY(by[e3d[e][0]]),
										BTS_MFX(bx[e3d[e][1]]), BTS_MFY(by[e3d[e][1]]), bHalf);
					} else {
						for (A_long e = 0; e < 4; ++e) {
							A_long n = (e + 1) & 3;
							BTS_CovLine(&cbuf, BTS_MFX(bx[e]), BTS_MFY(by[e]),
										BTS_MFX(bx[n]), BTS_MFY(by[n]), bHalf);
						}
					}
					BTS_FLUSH_LAYER(infoP->mKind[i], infoP->mLabel[i], infoP->bColR, infoP->bColG, infoP->bColB);
				}
				BTS_FLUSH_GROUP(infoP->bColR, infoP->bColG, infoP->bColB);
			}

			// 2b) box handles: 8 screen-aligned squares (4 corners + 4 edge mids).
			// Independent of box visibility (artistic choice) — needs only geometry.
			if (infoP->showHandles) {
				PF_FpLong hHalf = BTS_ScaleF(infoP->handleSize, dsx) * 0.5;
				for (A_long i = 0; i < infoP->markerCount; ++i) {
					if (!infoP->boxValid[i]) continue;
					PF_FpLong *bx = infoP->boxX[i], *by = infoP->boxY[i];
					if (infoP->box3D[i]) {
						// Mesh box: a handle on each of the 8 projected corners,
						// which is what AE itself shows for these layers.
						for (A_long c = 0; c < 8; ++c)
							BTS_CovSquare(&cbuf, BTS_MFX(bx[c]), BTS_MFY(by[c]), hHalf);
					} else {
						for (A_long e = 0; e < 4; ++e) {
							A_long n = (e + 1) & 3;
							PF_FpLong cxp = BTS_MFX(bx[e]), cyp = BTS_MFY(by[e]);				// corner
							PF_FpLong mxp = BTS_MFX((bx[e]+bx[n])*0.5), myp = BTS_MFY((by[e]+by[n])*0.5);	// edge mid
							BTS_CovSquare(&cbuf, cxp, cyp, hHalf);
							BTS_CovSquare(&cbuf, mxp, myp, hHalf);
						}
					}
					BTS_FLUSH_LAYER(infoP->mKind[i], infoP->mLabel[i], infoP->hColR, infoP->hColG, infoP->hColB);
				}
				BTS_FLUSH_GROUP(infoP->hColR, infoP->hColG, infoP->hColB);
			}

			// 2c) 3D local-axis arrows (X red, Y green, Z blue), own colors
			if (infoP->showAxes) {
				PF_FpLong axHalf  = BTS_ScaleF(infoP->axisWidth, dsx) * 0.5;
				// Head scales with shaft width so thick axes get proportionate heads.
				PF_FpLong headLen = BTS_ScaleF(infoP->axisWidth * 5, dsx);
				PF_FpLong headW   = headLen * 0.5;			// triangle half-base
				const PF_FpLong cosA = 0.866, sinA = 0.5;	// ~30 deg barbs
				A_Boolean tri = (infoP->axisHead == BTS_AHEAD_TRI);
				static const float axCol[3][3] = { {1.0f,0.25f,0.25f}, {0.30f,1.0f,0.30f}, {0.35f,0.55f,1.0f} };
				for (A_long k = 0; k < 3; ++k) {
					for (A_long i = 0; i < infoP->markerCount; ++i) {
						if (!infoP->axisValid[i]) continue;
						PF_FpLong aox = BTS_MFX(infoP->axisOX[i]),    aoy = BTS_MFY(infoP->axisOY[i]);
						PF_FpLong tx = BTS_MFX(infoP->axisEX[i][k]), ty = BTS_MFY(infoP->axisEY[i][k]);
						PF_FpLong dx = tx - aox, dy = ty - aoy, L = sqrt(dx*dx + dy*dy);
						if (tri && L > 1e-3) {
							// Cone/triangle head: shaft stops at the head base, then a
							// filled triangle from base corners to the tip.
							PF_FpLong ux = dx / L, uy = dy / L;			// tip direction
							PF_FpLong px = -uy, py = ux;				// perpendicular
							PF_FpLong baseX = tx - ux*headLen, baseY = ty - uy*headLen;
							BTS_CovLine(&cbuf, aox, aoy, baseX, baseY, axHalf);
							BTS_CovTriangle(&cbuf, tx, ty,
											baseX + px*headW, baseY + py*headW,
											baseX - px*headW, baseY - py*headW);
						} else {
							BTS_CovLine(&cbuf, aox, aoy, tx, ty, axHalf);
							// arrowhead: two barbs from the tip, back along the screen dir
							if (L > 1e-3) {
								PF_FpLong rx = -dx / L, ry = -dy / L;	// back toward origin
								PF_FpLong b1x = tx + headLen*(rx*cosA - ry*sinA), b1y = ty + headLen*(rx*sinA + ry*cosA);
								PF_FpLong b2x = tx + headLen*(rx*cosA + ry*sinA), b2y = ty + headLen*(-rx*sinA + ry*cosA);
								BTS_CovLine(&cbuf, tx, ty, b1x, b1y, axHalf);
								BTS_CovLine(&cbuf, tx, ty, b2x, b2y, axHalf);
							}
						}
					}
					BTS_Composite(outputP, depth, &cbuf, axCol[k][0], axCol[k][1], axCol[k][2], a);
				}
			}

			// 3) one marker per layer at its current-time Position
			if (infoP->showMarkers) {
				PF_FpLong E    = BTS_ScaleF(infoP->markerSize, dsx);
				PF_FpLong armX = BTS_ScaleF(infoP->markerSize, dsx),   armY = BTS_ScaleF(infoP->markerSize, dsy);
				PF_FpLong boxX = BTS_ScaleF(infoP->markerSize/2, dsx), boxY = BTS_ScaleF(infoP->markerSize/2, dsy);
				for (A_long i = 0; i < infoP->markerCount; ++i) {
					PF_FpLong sx = BTS_MFX(infoP->markerX[i]), sy = BTS_MFY(infoP->markerY[i]);
					switch (infoP->markerStyle) {
						case BTS_MSTYLE_CROSS: BTS_CovCrosshair(&cbuf, sx, sy, armX, armY, boxX, boxY, 0.5); break;
						case BTS_MSTYLE_DOT:   BTS_CovDot(&cbuf, sx, sy, E); break;
						case BTS_MSTYLE_ANCHOR:
						default:               BTS_CovAnchor(&cbuf, sx, sy, E); break;
					}
					BTS_FLUSH_LAYER(infoP->mKind[i], infoP->mLabel[i], infoP->mColR, infoP->mColG, infoP->mColB);
				}
				BTS_FLUSH_GROUP(infoP->mColR, infoP->mColG, infoP->mColB);
			}

			free(cbuf.cov);
		}
		#undef BTS_MFX
		#undef BTS_MFY
		#undef BTS_FLUSH_LAYER
		#undef BTS_FLUSH_GROUP
	}

	suites.HandleSuite1()->host_unlock_handle(reinterpret_cast<PF_Handle>(extra->input->pre_render_data));
	return err;
}

/* =========================================================================
   Entry points
   ========================================================================= */

extern "C" DllExport
PF_Err PluginDataEntryFunction2(
	PF_PluginDataPtr inPtr,
	PF_PluginDataCB2 inPluginDataCallBackPtr,
	SPBasicSuite* inSPBasicSuitePtr,
	const char* inHostName,
	const char* inHostVersion)
{
	PF_Err result = PF_Err_INVALID_CALLBACK;
	result = PF_REGISTER_EFFECT_EXT2(
		inPtr, inPluginDataCallBackPtr,
		"BTS Overlay", "aldai BTSOverlay", "Learning",
		AE_RESERVED_INFO, "EffectMain", "https://www.adobe.com");
	return result;
}

PF_Err
EffectMain(
	PF_Cmd cmd, PF_InData *in_data, PF_OutData *out_data,
	PF_ParamDef *params[], PF_LayerDef *output, void *extra)
{
	PF_Err err = PF_Err_NONE;
	try {
		switch (cmd) {
			case PF_Cmd_ABOUT:
				err = About(in_data, out_data, params, output);
				break;
			case PF_Cmd_GLOBAL_SETUP:
				err = GlobalSetup(in_data, out_data, params, output);
				break;
			case PF_Cmd_PARAMS_SETUP:
				err = ParamsSetup(in_data, out_data, params, output);
				break;
			case PF_Cmd_UPDATE_PARAMS_UI:
				err = UpdateParamsUI(in_data, out_data, params);
				break;
			case PF_Cmd_USER_CHANGED_PARAM:
				err = UserChangedParam(in_data, out_data, params,
									   reinterpret_cast<PF_UserChangedParamExtra*>(extra));
				break;
			case PF_Cmd_SMART_PRE_RENDER:
				err = PreRender(in_data, out_data, reinterpret_cast<PF_PreRenderExtra*>(extra));
				break;
			case PF_Cmd_SMART_RENDER:
				err = SmartRender(in_data, out_data, reinterpret_cast<PF_SmartRenderExtra*>(extra));
				break;
		}
	}
	catch (PF_Err &thrown_err) {
		err = thrown_err;
	}
	return err;
}
