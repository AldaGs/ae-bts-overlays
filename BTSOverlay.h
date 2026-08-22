/*
	BTSOverlay.h

	BTS Overlay — draws behind-the-scenes production gizmos (null markers +
	motion paths) onto the composite so a clean render can be exported *with*
	stylized overlays, for BTS / process videos.

	Form factor: a SmartFX effect applied to an ADJUSTMENT LAYER pinned at the
	top of the comp. Param 0 is the clean composite-below; we draw overlays on
	top and write pixels only — AE's render queue does the export.

	BUILD 1 (this file): lean SmartFX PASSTHROUGH + AEGP registration handshake
	+ diagnostic hooks (View popup, "Dump Diagnostics" button/CSV writer). No
	scene reading or drawing yet — those arrive in build 2.
*/

#pragma once

#ifndef BTSOVERLAY_H
#define BTSOVERLAY_H

typedef unsigned char		u_char;
typedef unsigned short		u_short;
typedef unsigned short		u_int16;
typedef unsigned long		u_long;
typedef short int			int16;
#define PF_TABLE_BITS	12
#define PF_TABLE_SZ_16	4096

#define PF_DEEP_COLOR_AWARE 1	// make sure we get 16bpc pixels

#include "AEConfig.h"

#ifdef AE_OS_WIN
	typedef unsigned short PixelType;
	#include <Windows.h>
#endif

#include "entry.h"
#include "AE_Effect.h"
#include "AE_EffectCB.h"
#include "AE_Macros.h"
#include "Param_Utils.h"
#include "AE_EffectCBSuites.h"
#include "String_Utils.h"
#include "AE_GeneralPlug.h"
#include "AEFX_ChannelDepthTpl.h"
#include "AEGP_SuiteHandler.h"

#include "BTSOverlay_Strings.h"

/* Versioning */
#define	MAJOR_VERSION	1
#define	MINOR_VERSION	0
#define	BUG_VERSION		0
#define	STAGE_VERSION	PF_Stage_DEVELOP
#define	BUILD_VERSION	1

/* Dev-only UI. The View popup (dumps an intermediate buffer instead of the
   final frame) and the Dump Diagnostics button (writes the per-layer CSV) are
   development instruments, not controls anyone shipping a cut should see. Set
   this to 1 to bring both back in the Effect Controls panel.

   They stay PRESENT as params either way - hidden, not deleted - so the param
   indices and disk IDs are untouched and no existing project breaks. */
#define BTS_DEV_UI		0

#if BTS_DEV_UI
	#define BTS_DEV_PUI		0
#else
	#define BTS_DEV_PUI		PF_PUI_INVISIBLE
#endif

/* View popup values (1-based, as AE reports popups). */
#define	BTS_VIEW_FINAL		1
#define	BTS_VIEW_DEBUG		2

/* Output mode. No longer a param - it is DERIVED in PreRender from the two
   Show Overlays / Overlays Only checkboxes, then carried through BTSInfo so
   SmartRender's three-way branch stays exactly as it was. */
#define	BTS_OUT_COMPOSITE	1	// overlays over the composite below
#define	BTS_OUT_OVERLAYS	2	// overlays only, on transparency
#define	BTS_OUT_INPUT		3	// clean composite below, no overlays

/* Gizmo Colors popup values - where every gizmo takes its color from. */
#define	BTS_COLSRC_GIZMO	1	// the per-gizmo color pickers below
#define	BTS_COLSRC_TYPE		2	// the per-layer-type swatches
#define	BTS_COLSRC_LABEL	3	// each layer's own AE label color

/* AE label slots. 0 is "No Label"; 1..16 are the real ones. Colors come from
   PF_AppGetColor(PF_App_Color_LABEL_0 + n), so they track whatever the user has
   set in Preferences > Labels rather than a hardcoded table. */
#define	BTS_LABEL_COUNT		16

/* Marker Style popup values. */
#define	BTS_MSTYLE_ANCHOR	1
#define	BTS_MSTYLE_CROSS	2
#define	BTS_MSTYLE_DOT		3

/* Arrowhead popup values. */
#define	BTS_AHEAD_BARBS		1
#define	BTS_AHEAD_TRI		2

/* Visibility popup values. */
#define	BTS_VIS_WHOLE		1
#define	BTS_VIS_WINDOW		2

/* Layer KINDS. Every gathered layer is classified into exactly one of these,
   in this precedence order (a null that is also an adjustment layer reads as a
   null, etc.). Drives the per-type show filter and the per-type color scheme. */
#define	BTS_LT_CAMERA		0
#define	BTS_LT_LIGHT		1
#define	BTS_LT_NULL			2
#define	BTS_LT_ADJUST		3
#define	BTS_LT_TEXT			4
#define	BTS_LT_SHAPE		5
#define	BTS_LT_PRECOMP		6
#define	BTS_LT_SOLID		7
#define	BTS_LT_FOOTAGE		8	// catch-all AV
#define	BTS_LT_MODEL		9	// IMPORTED 3D model (.glb) - never gets a box, see BTS_ReadBox
#define	BTS_LT_PRIMITIVE	10	// AE parametric mesh (cube/sphere/...) - gets a real 8-corner box
#define	BTS_LT_COUNT		11

/* Parameter order. MUST match the PF_ADD_* order in ParamsSetup.
   Topics (PF_ADD_TOPIC/PF_END_TOPIC) each occupy a param slot. */
enum {
	BTS_INPUT = 0,			// index 0 is always the input (composite-below)
	BTS_VIEW,				// popup: Final | Debug Values
	BTS_SHOW_OVERLAYS,		// checkbox: master A/B - off passes the clean input through
	BTS_OVERLAYS_ONLY,		// checkbox: draw on transparency instead of the composite
	BTS_OPACITY,			// master overlay opacity 0..100
	BTS_COLOR_SOURCE,		// popup: Per Gizmo | By Layer Type | By Label
	BTS_TIMING_TOPIC,		// --- Overlay Timing ---
	BTS_VISIBILITY,			// popup: Whole Layer | Timed Window
	BTS_WIN_START,			// window start (sec)
	BTS_WIN_LEN,			// window length (sec)
	BTS_TIMING_TOPIC_END,
	BTS_MARK_TOPIC,			// --- Layer Markers ---
	BTS_SHOW_MARKERS,		// checkbox
	BTS_MARKER_STYLE,		// popup: Anchor | Crosshair | Dot
	BTS_MARKER_COLOR,		// color
	BTS_MARKER_SIZE,		// slider (comp px; anchor ring radius / crosshair arm)
	BTS_SHOW_AXES,			// checkbox: 3D local-axis arrows (X/Y/Z)
	BTS_AXIS_LEN,			// slider (world/comp units)
	BTS_AXIS_WIDTH,			// slider (comp px): axis shaft thickness
	BTS_AXIS_HEAD,			// popup: Barbs | Triangle (cone)
	BTS_MARK_TOPIC_END,
	BTS_PATH_TOPIC,			// --- Motion Paths ---
	BTS_SHOW_PATHS,			// checkbox
	BTS_PATH_COLOR,			// color
	BTS_PATH_THICK,			// slider (comp px)
	BTS_SHOW_KFDOTS,		// checkbox: keyframe dots
	BTS_SHOW_TANGENTS,		// checkbox: spatial tangent handles
	BTS_SHOW_FRAMEDOTS,		// checkbox: per-frame velocity dots
	BTS_PATH_TOPIC_END,
	BTS_BOX_TOPIC,			// --- Bounding Boxes ---
	BTS_SHOW_BOXES,			// checkbox
	BTS_BOX_COLOR,			// color
	BTS_BOX_THICK,			// slider (comp px)
	BTS_SHOW_HANDLES,		// checkbox: 8 corner/edge handle squares
	BTS_HANDLE_COLOR,		// color
	BTS_HANDLE_SIZE,		// slider (comp px)
	BTS_BOX_TOPIC_END,
	BTS_TYPE_TOPIC,			// --- Layer Types ---
	BTS_SHOW_CAMERAS,	BTS_COL_CAMERAS,
	BTS_SHOW_LIGHTS,	BTS_COL_LIGHTS,
	BTS_SHOW_NULLS,		BTS_COL_NULLS,
	BTS_SHOW_ADJUST,	BTS_COL_ADJUST,
	BTS_SHOW_TEXT,		BTS_COL_TEXT,
	BTS_SHOW_SHAPES,	BTS_COL_SHAPES,
	BTS_SHOW_PRECOMPS,	BTS_COL_PRECOMPS,
	BTS_SHOW_SOLIDS,	BTS_COL_SOLIDS,
	BTS_SHOW_FOOTAGE,	BTS_COL_FOOTAGE,
	BTS_SHOW_MODELS,	BTS_COL_MODELS,
	BTS_SHOW_PRIMS,	BTS_COL_PRIMS,
	BTS_SHOW_GUIDES,		// checkbox: include guide layers (attribute filter)
	BTS_SHOW_LOCKED,		// checkbox: include locked layers
	BTS_TYPE_TOPIC_END,
	BTS_DUMP,				// button: write a diagnostics CSV
	BTS_NUM_PARAMS
};

/* Stable IDs saved in the project file. EXPLICIT + FROZEN: never change an
   existing number, never insert — ONLY append new params with the next number.
   (The param ORDER in ParamsSetup can change freely; these numbers must not.) */
enum {
	VIEW_DISK_ID			= 1,
	DUMP_DISK_ID			= 2,
	OPACITY_DISK_ID			= 3,
	MARK_TOPIC_DISK_ID		= 4,
	SHOW_MARKERS_DISK_ID	= 5,
	MARKER_COLOR_DISK_ID	= 6,
	MARKER_SIZE_DISK_ID		= 7,
	MARK_TOPIC_END_DISK_ID	= 8,
	PATH_TOPIC_DISK_ID		= 9,
	SHOW_PATHS_DISK_ID		= 10,
	PATH_COLOR_DISK_ID		= 11,
	PATH_THICK_DISK_ID		= 12,
	PATH_TOPIC_END_DISK_ID	= 13,
	SHOW_KFDOTS_DISK_ID		= 14,
	SHOW_TANGENTS_DISK_ID	= 15,
	SHOW_FRAMEDOTS_DISK_ID	= 16,
	MARKER_STYLE_DISK_ID	= 17,
	BOX_TOPIC_DISK_ID		= 18,
	SHOW_BOXES_DISK_ID		= 19,
	BOX_COLOR_DISK_ID		= 20,
	BOX_THICK_DISK_ID		= 21,
	SHOW_HANDLES_DISK_ID	= 22,
	HANDLE_COLOR_DISK_ID	= 23,
	HANDLE_SIZE_DISK_ID		= 24,
	BOX_TOPIC_END_DISK_ID	= 25,
	SHOW_AXES_DISK_ID		= 26,	// appended (was inserted mid-enum -> caused reset)
	AXIS_LEN_DISK_ID		= 27,
	TIMING_TOPIC_DISK_ID	= 28,
	VISIBILITY_DISK_ID		= 29,
	WIN_START_DISK_ID		= 30,
	WIN_LEN_DISK_ID			= 31,
	TIMING_TOPIC_END_DISK_ID = 32,
	AXIS_WIDTH_DISK_ID		= 33,
	AXIS_HEAD_DISK_ID		= 34,
	TYPE_TOPIC_DISK_ID		= 35,
	COLOR_BY_TYPE_DISK_ID	= 36,	// RETIRED (was the Color By Type checkbox). Never reuse.
	SHOW_CAMERAS_DISK_ID	= 37,
	COL_CAMERAS_DISK_ID		= 38,
	SHOW_LIGHTS_DISK_ID		= 39,
	COL_LIGHTS_DISK_ID		= 40,
	SHOW_NULLS_DISK_ID		= 41,
	COL_NULLS_DISK_ID		= 42,
	SHOW_ADJUST_DISK_ID		= 43,
	COL_ADJUST_DISK_ID		= 44,
	SHOW_TEXT_DISK_ID		= 45,
	COL_TEXT_DISK_ID		= 46,
	SHOW_SHAPES_DISK_ID		= 47,
	COL_SHAPES_DISK_ID		= 48,
	SHOW_PRECOMPS_DISK_ID	= 49,
	COL_PRECOMPS_DISK_ID	= 50,
	SHOW_SOLIDS_DISK_ID		= 51,
	COL_SOLIDS_DISK_ID		= 52,
	SHOW_FOOTAGE_DISK_ID	= 53,
	COL_FOOTAGE_DISK_ID		= 54,
	SHOW_GUIDES_DISK_ID		= 55,
	SHOW_LOCKED_DISK_ID		= 56,
	TYPE_TOPIC_END_DISK_ID	= 57,
	SHOW_MODELS_DISK_ID		= 58,
	COL_MODELS_DISK_ID		= 59,
	OUTPUT_DISK_ID			= 60,	// RETIRED (was the Output popup). Never reuse.
	SHOW_OVERLAYS_DISK_ID	= 61,
	OVERLAYS_ONLY_DISK_ID	= 62,
	COLOR_SOURCE_DISK_ID	= 63,
	SHOW_PRIMS_DISK_ID	= 64,
	COL_PRIMS_DISK_ID		= 65,
};

/* Per-render snapshot carried PreRender -> SmartRender via a PF_Handle.
   Minimal for build 1; grows as builds add scene geometry. */
#define BTS_MAX_MARKERS		1024
#define BTS_MAX_PATHS		64		// only animated layers get a path
#define BTS_PATH_SAMPLES	256		// dense sub-frame samples for the smooth line
#define BTS_MAX_FRAMES		512		// per-frame velocity-dot samples
#define BTS_MAX_KF			32		// keyframes stored per path

typedef struct BTSInfo {
	A_long		viewMode;		// BTS_VIEW_FINAL / BTS_VIEW_DEBUG
	A_long		outputMode;	// BTS_OUT_*
	PF_FpLong	downsampleX;	// in_data->downsample_x.num / .den
	PF_FpLong	downsampleY;
	A_long		width;			// input dims (pixels, at render res)
	A_long		height;
	// --- MVP style snapshot (from params, in PreRender) ----------------------
	PF_FpLong	opacity;		// 0..1 master overlay opacity
	A_Boolean	showMarkers;
	A_Boolean	showPaths;
	A_Boolean	showKfDots;
	A_Boolean	showTangents;
	A_Boolean	showFrameDots;
	float		mColR, mColG, mColB;	// marker color (straight, 0..1)
	float		pColR, pColG, pColB;	// path color
	A_long		markerStyle;	// BTS_MSTYLE_*
	A_long		markerSize;		// anchor ring radius / crosshair arm, FULL-res comp px
	A_Boolean	showAxes;
	A_long		axisLen;		// 3D axis arrow length, world units
	A_long		axisWidth;		// 3D axis shaft thickness, FULL-res comp px
	A_long		axisHead;		// BTS_AHEAD_BARBS / BTS_AHEAD_TRI
	A_Boolean	showBoxes;
	float		bColR, bColG, bColB;	// box color
	A_long		boxThick;		// box line thickness, FULL-res comp px
	A_Boolean	showHandles;
	float		hColR, hColG, hColB;	// handle color
	A_long		handleSize;		// handle square side, FULL-res comp px
	A_long		pathThick;		// path line thickness, FULL-res comp px
	// --- build 4: one crosshair per layer, read from the scene in PreRender ---
	// --- layer-type discriminators ------------------------------------------
	A_long		colorSource;			// BTS_COLSRC_*
	// Label colors, snapshotted on the MAIN thread (PFAppSuite is not safe to
	// call from the render threads PreRender runs on). Index 0 = No Label.
	float		labelCol[BTS_LABEL_COUNT + 1][3];
	A_Boolean	showType[BTS_LT_COUNT];	// per-kind filter
	float		tCol[BTS_LT_COUNT][3];	// per-kind color (straight, 0..1)
	A_long		markerCount;			// # valid entries below
	PF_FpLong	markerX[BTS_MAX_MARKERS];	// layer Position, FULL-res comp px
	PF_FpLong	markerY[BTS_MAX_MARKERS];
	A_long		mKind[BTS_MAX_MARKERS];		// BTS_LT_* per marker
	A_long		mLabel[BTS_MAX_MARKERS];	// AE label slot per marker (0 = none)
	// --- bounding box, parallel to markers (4 transformed corners, comp px) ---
	A_Boolean	boxValid[BTS_MAX_MARKERS];
	A_Boolean	box3D[BTS_MAX_MARKERS];	// TRUE = 8-corner mesh box, FALSE = 4-corner bounds
	PF_FpLong	boxX[BTS_MAX_MARKERS][8];	// [0..3] back face, [4..7] front face when box3D
	PF_FpLong	boxY[BTS_MAX_MARKERS][8];
	// --- 3D local-axis arrows (projected origin + 3 endpoints), per marker ----
	A_Boolean	axisValid[BTS_MAX_MARKERS];
	PF_FpLong	axisOX[BTS_MAX_MARKERS], axisOY[BTS_MAX_MARKERS];	// origin (anchor)
	PF_FpLong	axisEX[BTS_MAX_MARKERS][3], axisEY[BTS_MAX_MARKERS][3];	// X,Y,Z tips
	// --- build 5: sampled motion path per animated layer (comp px) -----------
	A_long		pathCount;					// # animated layers with a path
	A_long		pathKind[BTS_MAX_PATHS];	// BTS_LT_* per path
	A_long		pathLabel[BTS_MAX_PATHS];	// AE label slot per path (0 = none)
	A_long		pathNPts[BTS_MAX_PATHS];	// dense line vertices (smooth curve)
	PF_FpLong	pathX[BTS_MAX_PATHS][BTS_PATH_SAMPLES];
	PF_FpLong	pathY[BTS_MAX_PATHS][BTS_PATH_SAMPLES];
	// --- per-frame velocity dots (spacing = speed) --------------------------
	A_long		frameN[BTS_MAX_PATHS];
	PF_FpLong	frameX[BTS_MAX_PATHS][BTS_MAX_FRAMES];
	PF_FpLong	frameY[BTS_MAX_PATHS][BTS_MAX_FRAMES];
	// --- keyframe dots + spatial tangent handles, parallel to paths (comp px) ---
	A_long		kfN[BTS_MAX_PATHS];			// keyframes on path p
	PF_FpLong	kfX[BTS_MAX_PATHS][BTS_MAX_KF];
	PF_FpLong	kfY[BTS_MAX_PATHS][BTS_MAX_KF];
	PF_FpLong	kfInX[BTS_MAX_PATHS][BTS_MAX_KF];	// in-tangent handle endpoint
	PF_FpLong	kfInY[BTS_MAX_PATHS][BTS_MAX_KF];
	PF_FpLong	kfOutX[BTS_MAX_PATHS][BTS_MAX_KF];	// out-tangent handle endpoint
	PF_FpLong	kfOutY[BTS_MAX_PATHS][BTS_MAX_KF];
} BTSInfo, *BTSInfoP, **BTSInfoH;

extern "C" {

	DllExport
	PF_Err
	EffectMain(
		PF_Cmd			cmd,
		PF_InData		*in_data,
		PF_OutData		*out_data,
		PF_ParamDef		*params[],
		PF_LayerDef		*output,
		void			*extra);

}

#endif // BTSOVERLAY_H
