"use strict";
/**
 * Performance-testing deck generator: builds the two decks of the VSX3000 performance-testing procedure
 * from their content files. Adapted from the registration generator (make_registration_deck.js of
 * plane_plane_registration_ro), which was itself adapted from the stage-1 generator of
 * depth_calibration_from_spherical_target.
 *
 *   procurement and build deck : build/perf_build_deck_content.json     -> vsx3000_procurement_build.pptx
 *   test procedure deck        : build/perf_procedure_deck_content.json -> vsx3000_test_procedure.pptx
 *
 * Each content JSON is the single source of truth for that deck's text and speaker notes; the figures
 * are read from docs/presentations/assets/*.png (made by build/make_assets.py; pixel sizes are read at
 * build time; a figure file that is absent is drawn as a labeled placeholder, see loadImage). The only text in this script is the few labels of EXTRA_LABELS (diagram legends) and the
 * figures' alternative text.
 *
 * Run from the repository root:
 *     NODE_PATH=<folder with node_modules for pptxgenjs, react-icons, react, react-dom, sharp> \
 *         node docs/presentations/build/make_perf_deck.js                      # builds both decks
 *     NODE_PATH=... node docs/presentations/build/make_perf_deck.js <content.json> <output.pptx>   # builds one
 *
 * Structure (see the pptx skill, "Structured decks"):
 *   - a named theme ("VSX3000 Performance") whose colors are written into the file by applyTheme();
 *   - every color is a theme (scheme) color, except inside rasterized icons (images need hex);
 *   - two layouts: TITLE_DARK and TITLE_ONLY, with named placeholders that slides fill by name;
 *   - one section per topic (each slide's "section" field); speaker notes on every slide.
 *
 * Slides are built by the "builder" field of each slide in the content JSON (falling back to its "id"; see
 * `builders` below). Several slides share one builder: fixtures and runout (figure card and icon cards),
 * board (image and check rows), approach (image and numbered steps), plate_spec (table, with the plate
 * drawing only beside a two-column table), checks, loop, robot_program and prep (columns over a message),
 * and "pair" (two drawings side by side, new in this generator).
 *
 * All sizes are inches unless a name says "PT" (points). Nothing is hard-coded in the slide
 * builders: every dimension, font size, spacing and color comes from the constants below.
 */

const fs = require("fs");
const path = require("path");
const pptxgen = require("pptxgenjs");
const React = require("react");
const ReactDOMServer = require("react-dom/server");
const sharp = require("sharp");
const fa = require("react-icons/fa");

// ---------------------------------------------------------------------------------------------
// File locations (relative to the repository root, which is the working directory)
// ---------------------------------------------------------------------------------------------
const PRESENTATIONS_DIR = path.join("docs", "presentations");
// The two decks built when no command-line arguments are given: [content JSON, output pptx].
const DECKS = [
	[path.join(PRESENTATIONS_DIR, "build", "perf_build_deck_content.json"), path.join(PRESENTATIONS_DIR, "vsx3000_procurement_build.pptx")],
	[path.join(PRESENTATIONS_DIR, "build", "perf_procedure_deck_content.json"), path.join(PRESENTATIONS_DIR, "vsx3000_test_procedure.pptx")],
];
// Folder of the pptx skill's scripts; only apply_theme.js is used from it.
const SKILL_SCRIPTS_DIR =
	process.env.PPTX_SKILL_SCRIPTS ||
	"/root/.claude/skills/synced/f2fe48f6-69a7-4a19-a27a-2fac391c7f33_06dd8df9-d222-4305-b701-72191c02d7f8/pptx/scripts";
const { applyTheme } = require(path.join(SKILL_SCRIPTS_DIR, "apply_theme.js"));

// ---------------------------------------------------------------------------------------------
// Theme
// ---------------------------------------------------------------------------------------------
const THEME = {
	name: "VSX3000 Performance",
	headFontFace: "Cambria", // headings
	bodyFontFace: "Calibri", // everything else
	colors: {
		dk1: "1F2A30", // graphite text
		lt1: "FFFFFF", // white
		dk2: "2E4A52", // deep steel teal: dark slides, headings
		lt2: "EEF2F3", // cool light gray tint: cards
		accent1: "D9731A", // safety orange: step badges, stat numbers, key message
		accent2: "3E7C87", // steel teal: icons, secondary
		accent3: "8FA9AE", // soft steel
		accent4: "B23A2E", // signal red: warnings only
		accent5: "5B6B70", // muted caption gray
		accent6: "C9D6D8", // pale steel: outlines
		hlink: "3E7C87",
		folHlink: "5B6B70",
	},
};

// Theme font references, so that text follows the theme instead of naming a font.
const DECK_SUBJECT = "VSX3000 performance testing"; // document property shared by both decks
const DECK_AUTHOR = "Sensor performance estimation"; // document property shared by both decks
const HEAD_FONT_REF = "+mj-lt"; // theme heading font (Cambria)
const MONO_FONT = "Courier New"; // the one explicitly named font: the pose-log example line (manifest slide)

// ---------------------------------------------------------------------------------------------
// Page geometry
// ---------------------------------------------------------------------------------------------
const SLIDE_W = 13.333; // LAYOUT_WIDE width
const SLIDE_H = 7.5; // LAYOUT_WIDE height
const MARGIN = 0.5; // minimum distance from content to slide edge
const GAP = 0.3; // standard gap between blocks
const GAP_TIGHT = 0.2; // gap between items inside one list or column (rows of a list, icon to text)
const CONTENT_X = MARGIN; // left edge of content
const CONTENT_W = SLIDE_W - 2 * MARGIN; // width of content area

const TITLE_Y = MARGIN; // content-slide title: top
const TITLE_H = 0.75; // content-slide title: height (one line at TITLE_PT)
const CONTENT_TOP = TITLE_Y + TITLE_H + GAP; // first y available to content

const NUMBER_W = 0.6; // slide-number box width
const NUMBER_H = 0.3; // slide-number box height
const NUMBER_BOTTOM_GAP = 0.2; // distance of slide-number box from the slide's bottom edge
const NUMBER_Y = SLIDE_H - NUMBER_BOTTOM_GAP - NUMBER_H; // slide-number box top
const CONTENT_BOTTOM = NUMBER_Y - GAP; // last y available to content
const CONTENT_H = CONTENT_BOTTOM - CONTENT_TOP; // height available to content

// Dark layout: title top-left as on content slides but larger, then subtitle, footer at the bottom.
const DARK_TITLE_H = 1.6; // title placeholder height: two lines at DARK_TITLE_PT (titles run to one or two lines; the text sits at the bottom of the box, next to the subtitle)
const DARK_SUBTITLE_Y = MARGIN + DARK_TITLE_H + GAP_TIGHT; // subtitle top
const DARK_SUBTITLE_W = 9.0; // subtitle width (wraps to two lines)
const DARK_SUBTITLE_H = 0.9; // subtitle height
const DARK_FOOTER_H = 0.4; // footer placeholder height
const DARK_FOOTER_Y = SLIDE_H - MARGIN - DARK_FOOTER_H; // footer top

// ---------------------------------------------------------------------------------------------
// Typography (points)
// ---------------------------------------------------------------------------------------------
const TITLE_PT = 34; // content-slide title (titles are kept to about 54 characters so one line fits)
const TITLE_MIN_PT = 30; // smallest content-slide title (a long title is stepped down to it)
const TITLE_STEP_PT = 2; // step of that reduction
const DARK_TITLE_PT = 44; // dark-slide title
const DARK_SUBTITLE_PT = 22; // title-slide subtitle
const FOOTER_PT = 14; // title-slide footer
const NUMBER_PT = 12; // slide number
const HEAD_PT = 20; // column and card heads that are the main label
const CARD_HEAD_PT = 18; // card heads
const BODY_PT = 16; // comfortable body text
const BODY_MIN_PT = 14; // smallest body text allowed
const CAPTION_PT = 12; // captions (never below 11)
const LABEL_PT = 14; // stat labels
const MESSAGE_PT = 22; // key message line
const BOX_MESSAGE_PT = 18; // key message inside a box
const BADGE_NUM_PT = 18; // number inside a numbered circle
const SMALL_BADGE_NUM_PT = 14; // number inside a small numbered circle
const DIAGRAM_LABEL_PT = 12; // labels inside the bootstrap diagram
const MONO_PT = 14; // pose-log example line (14 pt: the 88-character example still fits on one line)
const STAT_PT = 54; // large stat value
const STAT_MED_PT = 44; // medium stat value
const STAT_SMALL_PT = 36; // small stat value
const STAT_PRICE_PT = 32; // stat value that is a price range
const STAT_PRICE_LONG_PT = 26; // a price range of two four-figure amounts, which must stay on one line in its box
const BODY_LINE_FACTOR = 1.2; // line height as a multiple of the font size
const BULLET_INDENT_PT = 14; // hanging indent of bullets
const PARA_SPACE_PT = 6; // space after each bullet paragraph (default)
const PARA_SPACE_LOOSE_PT = 12; // space after each bullet paragraph in roomy cards
const NBSP = "\u00a0"; // non-breaking space: keeps a number and its unit on one line
// Units that must stay attached to the number before them (matched after a digit and a space).
const UNIT_PATTERN = /(\d) (mm|min|s|h|percent|degrees|degree C|inch)\b/g;

// Rough text-width model used only to size containers (Calibri/Cambria average character width
// as a fraction of the font size). Deliberately a little wide so that boxes are never too small.
const CHAR_W_REGULAR = 0.46;
const CHAR_W_BOLD = 0.5;
const TEXT_SLACK = 0.06; // extra height added to every estimated text block (inches)

// ---------------------------------------------------------------------------------------------
// Shared component sizes
// ---------------------------------------------------------------------------------------------
const CARD_PAD = 0.25; // inner padding of a card
const CARD_RADIUS = 0.12; // corner radius of cards (inches)
const OUTLINE_PT = 1; // outline weight of white cards
const BADGE_D = 0.55; // numbered circle diameter
const BADGE_SMALL_D = 0.45; // small circle diameter (list rows)
const ICON_BADGE_D = 0.7; // large icon circle diameter
const ICON_GLYPH_RATIO = 0.5; // glyph size as a fraction of its circle
const ICON_RASTER_PX = 256; // icon raster size (>= 256)
const ARROW_GAP = 0.6; // gap between cards that hold an arrow
const ARROW_W = 0.25; // chevron width
const ARROW_H = 0.4; // chevron height
const FLOW_LINE_PT = 2; // weight of connector lines
const FLOW_ROW_GAP = 0.5; // gap between the two rows of the flow slide
const ARROWHEAD_W = 0.3; // down-pointing arrowhead width
const ARROWHEAD_H = 0.2; // down-pointing arrowhead height
const TABLE_BORDER_PT = 0.5; // table rule weight

// ---------------------------------------------------------------------------------------------
// Per-slide settings
// ---------------------------------------------------------------------------------------------
// Title slide graphic: sensor and three tilted boards at three standoffs (see TITLE_ART below)
const TITLE_ART = {
	// Schematic side view of the capture geometry: the sensor at the left looking right at a target (a thin plate seen
	// edge-on) at three distances, tilted 0, 15 and 30 degrees, as the plan's stations and tilts do (kept from the
	// registration deck; it illustrates the geometry, it is not to scale). All sizes in inches.
	sensorW: 1.1, // sensor body width
	sensorH: 1.5, // sensor body height
	lensD: 0.5, // lens circle diameter
	lensInset: 0.15, // distance of the lens circle from the sensor's right face
	plateW: 0.14, // board thickness as drawn (edge-on)
	plateH: 2.3, // board height as drawn
	standoffsMm: [550, 750, 950], // the plan's three standoffs (mm), from the sensor's right face
	tiltsDeg: [0, 15, 30], // tilt of the board at each standoff (degrees)
	inchesPerMm: 0.0042, // drawing scale along the viewing direction
	originD: 0.24, // diameter of the tool-frame origin dot at each board's center
	bottomGap: GAP, // gap between graphic baseline and footer
	outlinePt: 2, // outline weight of the shapes
	plateRadius: 0.04, // board corner radius
};

const PRODUCT = {
	cardH: 2.6, // stat card height (2.8 in the registration generator; the body paragraph needs the height)
	badgeD: BADGE_D, // icon badge on each card
	valueH: 0.9, // height of the stat value line
	labelH: 0.8, // height of the stat label (three lines; 0.6 in the registration generator, whose labels ran to two)
	messageH: 0.8, // key-message row height (two lines at 22 pt; 0.7 in the registration generator)
	messageBadgeD: 0.6, // icon badge next to the message
	bodyPt: BODY_PT, // body text size (20 in the registration generator; this deck's paragraph is longer)
};

const FLOW = {
	numberD: BADGE_D,
	headPt: 16, // 18 in the registration generator; the head now shares a line with the badge, and "Series C and D" wraps to two lines in the room beside it
	textPt: BODY_MIN_PT,
};

// Fixtures, board_build and runout share one composition: a white card with the wide figure and its caption at the
// left, icon cards stacked at the right (figureWithCards). Card heights follow their text, so the right column's
// width decides whether the text fits; figureCardW is what is left for the figure.
const ICON_CARD_GAP = 0.15; // gap between stacked icon cards of fixtures and runout (0.2 in the registration generator; four long cards need the height)
const FIGURE_SHARE_WIDE = 0.55; // share of the content width taken by the figure column beside cards, points or steps (fixtures, runout, board_build, approach, board)
const FIGURE_CARD_ICON_D = 0.55;
const FIGURE_CARD_SIDE_PAD = 0.1;
const ICON_CARD_GAP_STEP = 0.01; // step by which the gap between icon cards shrinks when their text needs the height (figureWithCards)
const ICON_CARD_GAP_MIN = 0.05; // smallest gap between icon cards
const FIXTURES = {
	stackGapMin: ICON_CARD_GAP_MIN,
	iconD: FIGURE_CARD_ICON_D, // icon circle (ICON_BADGE_D in the registration generator; the wider figure column leaves the text less width)
	sidePad: FIGURE_CARD_SIDE_PAD, // padding at the left of the icon, between icon and text, and at the right of the text (CARD_PAD and GAP_TIGHT in the registration generator)
	headPt: 16, // card head (CARD_HEAD_PT, 18, in the registration generator; the four long cards of the targets slide need the line)
	textPt: BODY_MIN_PT,
	cardPad: 0.02, // padding above and below the text inside each card (0.1 in the registration generator; the wider figure column leaves the text less width, so more lines)
	textRightPad: 0, // extra space at the right of the card text
	stackGap: ICON_CARD_GAP, // gap between the icon cards
};

// Shared by every table slide (cost, suppliers, build_list, buy_list, plate_spec).
const TABLE_MIN_PT = 13; // smallest table text allowed (tables may be a point smaller than body text; used where a long table must fit)
const TABLE_HEAD_H = 0.5; // header row height
const TABLE_CELL_MARGIN = [0.05, 0.12, 0.05, 0.12]; // cell margins: top, right, bottom, left (inches)
const TABLE_ROW_LINE_FACTOR = BODY_LINE_FACTOR; // row height per text line, as a multiple of the font size
const TABLE_FIT_TOLERANCE = 0.01; // inches of slack before a table that is taller than its box is reported
const TABLE_CAPTION_GAP = GAP; // gap between a table (or its visual) and the caption under it

const COST = {
	tableW: 7.9, // table width
	colW: [5.6, 2.3], // table column widths (sum to tableW)
	cellPt: BODY_MIN_PT,
	statValuePt: STAT_PRICE_LONG_PT,
	statValueH: 0.6, // stat value line height
	statLabelH: 0.8, // stat label height (three lines; 0.6 in the registration generator)
};

// Figures that stand bare on the slide (no white card around them) get a thin outline of their own size.
const FIGURE_FRAME_PT = OUTLINE_PT; // outline weight of a bare figure's frame
const CAPTION_GAP = GAP_TIGHT; // gap between a figure and the caption under it
const CAPTION_LINE_H = 0.3; // height of a one-line caption (12 pt)
const CAPTION_TWO_LINE_H = 0.5; // height of a two-line caption
const STACK_GAP = GAP; // gap between stacked cards (step lists, card columns)

const BOARD_BUILD = {
	stackGapMin: ICON_CARD_GAP_MIN,
	iconD: FIGURE_CARD_ICON_D, // icon circle (ICON_BADGE_D in the registration generator; the wider figure column leaves the text less width)
	sidePad: FIGURE_CARD_SIDE_PAD, // padding at the left of the icon, between icon and text, and at the right of the text (CARD_PAD and GAP_TIGHT in the registration generator)
	headPt: CARD_HEAD_PT,
	textPt: BODY_MIN_PT,
	cardPad: 0.05, // padding above and below the text inside each card (0.1 in the registration generator; the wider figure column leaves the text less width, so more lines)
	textRightPad: 0, // extra space at the right of the card text
	stackGap: GAP_TIGHT,
};

const SUPPLIERS = {
	colW: [3.1, 5.2, 4.033], // Item, Candidate suppliers, Notes (sums to CONTENT_W)
	cellPt: BODY_MIN_PT, // 14 pt fits; the table never goes below it
};

// "What must be built": table on the left, a tint card with a large toolbox icon on the right.
const BUILD_LIST = {
	colW: [4.9, 2.1, 1.6, 0.9], // Item, Quantity, Drawing, From (the registration generator had 4.6, 1.2, 1.9, 1.5: ten rows and a two-line caption leave 4.35 in, which only 13 pt and mostly one-line rows fit)
	cellPt: TABLE_MIN_PT, // 13 pt: ten rows at 14 pt need more height than the slide has
	iconD: 1.6, // large teal icon circle in the visual card
};

// "What must be bought": the full content width; the cost column is right-aligned.
const BUY_LIST = {
	colW: [4.2, 4.333, 2.3, 1.5], // Item, Purpose, Estimated cost (USD), From stage 1 (sums to CONTENT_W)
	cellPt: BODY_MIN_PT,
};

// Plate purchase specification. Changed from the registration generator: the column widths depend on the number of
// table columns. A two-column table (Requirement, one plate) keeps the native-shape drawing of the plate beside it;
// a three-column table (Requirement, two kinds of target: the feature targets) takes the full content width and has
// no drawing, because the board-shaped plate drawing would not describe those targets.
const PLATE_SPEC = {
	colW: [1.7, 6.7], // Requirement, plate (two-column table, with the drawing)
	cellWidePt: TABLE_MIN_PT, // text size of the three-column table: seven rows of up to three lines do not fit the height at 14 pt
	colWWide: [2.0, 5.2, 5.133], // Requirement, first target group, second target group (three-column table, sums to CONTENT_W)
	cellPt: BODY_MIN_PT,
	scale: 0.015, // drawing scale in inches per mm (the 200 x 150 mm plate is drawn 3.0 x 2.25 in)
	plateWmm: 200, // plate width in the drawing (mm; the long edge)
	plateHmm: 150, // plate height in the drawing (mm; the short edge)
	outlinePt: 1.5, // outline weight of the plate
	edgePt: 5, // weight of the two highlighted locating edges
	padDmm: 15, // support-pad diameter (mm), as on drawing SC1-05
	// Support-pad centers (mm) from the plate center, x along the long edge, y up, as on drawing SC1-05:
	// two near the bottom corners and one at the top middle, each 8 mm in from the board edges.
	padsMm: [[-92, -67], [92, -67], [0, 67]],
	legendPt: BODY_MIN_PT, // legend text
	legendH: 0.35, // legend row height
	legendKeyW: 0.5, // length of a legend key (a short line, or a pad circle centered in the same width)
	legendGap: GAP_TIGHT, // gap between legend rows and between the drawing and the legend
};

const ACCEPTANCE = {
	listW: 8.7, // checklist column width (wide enough that the drawing number SC1-05 does not break at a hyphen)
	textPt: BODY_MIN_PT, // 16 in the registration generator; eight items, one of them long, need 14 pt to stay on two lines
	iconD: BADGE_SMALL_D,
	rowGap: 0.1, // gap between checklist rows (GAP_TIGHT in the registration generator, which had six items)
	badgeD: 2.6, // large clipboard-check circle on the right
};

// Six bootstrap captures: the sensor at the left, then the six frames in a grid of rows x columns, each frame
// with its text beside it (six frames in one row would leave about 1.5 in for each text).
const BOOTSTRAP = {
	columns: 3, // frames per row (six frames: two rows of three)
	frameW: 1.3, // width of each mini image frame
	frameH: 1.4, // height of each mini image frame
	rowGap: GAP_TIGHT, // gap between the two rows of frames
	textGap: GAP_TIGHT, // gap between a frame and its text
	sensorW: 1.2, // sensor rectangle width (as tall as the frame grid)
	bigD: 0.95, // circle diameter when the board faces the sensor
	farD: 0.7, // circle diameter when the board is farther (drawn smaller)
	tiltRatio: 0.7, // a tilted board is drawn this much narrower (schematic: a real 20 degree tilt foreshortens by only 6 percent)
	textPt: BODY_MIN_PT,
	crossPt: 1, // weight of the crosshair lines in each frame
	pointsPt: BODY_PT,
	sensorPt: 16, // "Sensor" label size
	lensD: 0.45, // lens circle on the sensor
	frameLineGap: 0.2, // margin of the crosshair from the frame edge
};

const PLAN = {
	imageTargetW: 8.6, // wanted figure width; the height available may allow less (a narrower figure leaves the stat labels room for three lines)
	captionH: CAPTION_TWO_LINE_H, // two-line caption under the figure
	valuePt: STAT_SMALL_PT,
	valueH: 0.6, // stat value line height
	labelPt: LABEL_PT,
	cardPad: GAP_TIGHT, // padding inside the narrow stat cards
};

const LOOP = {
	cardH: 3.1, // step cards (tall enough for the longest step text: seven lines at 14 pt in a narrow card; the head sits beside the badge, which is why 3.2 in the registration generator can be less)
	headPt: 16, // 18 in the registration generator; the head shares a line with the badge, and a long head wraps to two lines
	textPt: BODY_MIN_PT,
	statValuePt: STAT_PRICE_PT, // 32 pt keeps '~30 min' on one line in the narrow stat box
	statValueH: 0.6, // stat value line height
	statLabelH: 0.95, // stat label height (four lines in a narrow stat card; 0.6 in the registration generator)
	rowH: 1.8, // lower row (stats and message)
	messageBadgeD: ICON_BADGE_D,
};

const MANIFEST = {
	headBarH: 0.7, // filled header block inside each card
	headPt: CARD_HEAD_PT,
	bulletPt: BODY_PT, // two columns with five and six bullets: 16 pt with the tighter paragraph spacing keeps both inside their cards
	paraSpacePt: PARA_SPACE_PT,
	exampleBoxH: 0.8,
	exampleBadgeD: BADGE_SMALL_D + 0.1,
};

const CHECKS = {
	iconD: ICON_BADGE_D,
	headPt: CARD_HEAD_PT,
	flagPt: BODY_PT,
	textPt: BODY_MIN_PT, // four cards in a 2 x 2 grid, each with a head, a flag line and up to four lines of text: 14 pt fits
	cardPad: 0.15, // padding inside each card (GAP_TIGHT in the registration generator; tighter so the five-card slide fits)
	gap: GAP_TIGHT, // gap between card rows and between the grid and the caption (GAP in the registration generator; the five-card slide needs the height)
	perRow: { 4: [2, 2], 5: [3, 2] }, // cards per row, by card count (the grid was fixed at two columns in the registration generator)
};
// Icon and fill color (a THEME color key) of the cards of a checks slide, by slide id; a slide not listed gets warning triangles on accent4 (signal red, for things that go wrong).
const CHECKS_ICONS = {
	warmup: { icons: ["sliders", "clock", "move", "camera"], fill: "accent2" }, // configuration, warm-up, settle and vibration, intrinsics
	series_ab: { icons: ["gauge", "ruler", "bullseye", "edit"], fill: "accent2" }, // A noise, B edges, sentinels, optional
};

// Three columns of icon + head + bullets over one boxed key message (robot_program, prep).
const COLUMNS_MESSAGE = {
	iconD: 0.55, // column icon circle (ICON_BADGE_D in the registration generator; the bullets need the height)
	headPt: CARD_HEAD_PT, // 18 pt (HEAD_PT, 20, in the registration generator): a two-line head fits the head box
	headH: 0.6, // height of the head box, beside the icon (two lines at 18 pt)
	bulletGap: GAP_TIGHT / 2, // gap between the head row and the bullets (GAP_TIGHT in the registration generator)
	messageIconD: ICON_BADGE_D,
	bottomPad: 0.1, // padding under the bullets (CARD_PAD in the registration generator)
	messageGap: GAP_TIGHT, // gap between the columns and the message box (GAP in the registration generator)
	paraSpacePt: 4, // space after each bullet (PARA_SPACE_PT in the registration generator; this deck's columns have up to six bullets of two or three lines)
};
const ROBOT_PROGRAM = {
	bulletPt: BODY_MIN_PT, // three narrow columns, the outputs column with 13 lines at 16 pt: 14 pt keeps every column inside its card
	paraSpacePt: COLUMNS_MESSAGE.paraSpacePt, // tighter than the roomy default so the longest column fits
	messageH: 0.8, // the message runs one line
};

// Preparation slide: three columns of icon + head + bullets over the boxed message (same helper as robot_program).
const PREP = {
	bulletPt: BODY_MIN_PT, // three narrow columns with up to five bullets: 14 pt keeps every column inside its card
	paraSpacePt: COLUMNS_MESSAGE.paraSpacePt,
	messageH: 0.8, // message box height (the message runs two lines at 18 pt; a little less than the stage-1 0.9 gives the bullets room)
};

// Run-out fixture slide: the figure card at the left, four icon cards at the right (same composition as board_build).
const RUNOUT = {
	stackGapMin: ICON_CARD_GAP_MIN,
	iconD: FIGURE_CARD_ICON_D, // icon circle (ICON_BADGE_D in the registration generator; the wider figure column leaves the text less width)
	sidePad: FIGURE_CARD_SIDE_PAD, // padding at the left of the icon, between icon and text, and at the right of the text (CARD_PAD and GAP_TIGHT in the registration generator)
	headPt: CARD_HEAD_PT,
	textPt: BODY_MIN_PT,
	cardPad: 0.05, // padding above and below the text inside each card (0.1 in the registration generator; the wider figure column leaves the text less width, so more lines)
	textRightPad: 0, // extra space at the right of the card text
	stackGap: ICON_CARD_GAP,
};

// Slides with a figure beside a list (approach, board, residuals): the figure at the full content height at the left,
// or at this width when it is wide (the board figure is 2.6 times as wide as tall), the list beside it.
const IMAGE_BESIDE_MAX_W = FIGURE_SHARE_WIDE * CONTENT_W; // the least width of the figure beside points (55 percent of the content width; 6.8 in the registration generator)

// Figure with numbered steps beside it (approach).
const IMAGE_STEP_W = 0.1; // step by which the figure beside the steps narrows when the steps do not fit
const IMAGE_STEPS = {
	stackGapMin: ICON_CARD_GAP_MIN,
	stepPt: BODY_MIN_PT, // five steps, two of three lines: 14 pt keeps the stack inside the content height
	rowCardPad: 0.05, // padding inside a step card (0.1 in the registration generator; the wider figure leaves the steps less width)
	stackGap: GAP_TIGHT, // gap between step cards (tighter than GAP so five cards fit)
	get minRowH() {
		return BADGE_D + 2 * this.rowCardPad; // minimum step-card height
	},
};

// Figure with check rows beside it (board, residuals).
const IMAGE_POINTS = {
	pointPt: BODY_PT, // the size tried first; the next smaller one in pointPtFallbacks is used when the rows do not fit the height (changed from the registration generator, whose rows all fit at 16 pt)
	figureMaxW: 8.4, // widest the figure gets (changed from the registration generator, where it was IMAGE_BESIDE_MAX_W)
	pointPtFallbacks: [BODY_MIN_PT], // sizes tried after pointPt, largest first (never below the body minimum)
	rowGap: GAP_TIGHT, // gap between rows (five rows, some of three lines)
};

const SPOILERS = {
	columns: 2,
	rowsFirstColumn: 4, // 4 + 3 items
	textPt: BODY_PT,
	iconD: BADGE_D + 0.1,
	cardPad: GAP_TIGHT, // padding inside each item card (smaller than CARD_PAD so four rows fit)
	rowGap: GAP_TIGHT, // gap between rows (the left column's first two items run to three lines)
};

const DELIVERABLES = {
	listW: 8.3, // checklist column width
	textPt: BODY_MIN_PT, // seven items, one of three lines: 14 pt keeps the list inside the content height
	iconD: BADGE_SMALL_D,
	rowGap: GAP_TIGHT, // gap between checklist rows
	headPt: HEAD_PT,
	softwareIconD: ICON_BADGE_D,
	bulletPt: BODY_PT,
};

// Icons and figure alternative texts that are not content but belong to a slide, by slide id (the content files carry
// no icon for the cards of the figure-with-cards slides). A slide id that is not listed gets the default icons.
const CARD_ICONS = {
	targets: ["board", "layers", "crosshairs", "ruler"], // T2, T3a/T3b, T4/T5, the two gaps
	edges: ["crosshairs", "ruler", "layers", "paint"], // bevel, land, posts, finish
	mounting: ["wrench", "layers", "tool", "crosshairs"], // adapter, spigot, ball-lock pin, datum
	fixtures_bought: ["gauge", "layers", "wrench", "clock"], // run-out fixture, drift-run stand, ball-lock pins, temperature loggers
};
const DEFAULT_CARD_ICONS = ["board", "layers", "crosshairs", "ruler"];
const FIGURE_ALT = {
	targets: "Front views of the five targets T2, T3a, T3b, T4 and T5 to one scale",
	edges: "Cross-section of a cutout and of a disk on its post with the bevel at the knife edge",
	mounting: "Cross-section of the mounting stack: flange, adapter, spigot, ball-lock pin, back plate, standoffs, front plate",
	fixtures_bought: "The setup: the sensor on its rigid stand and the robot carrying a target on the adapter",
	adapter_drawing: "Drawing PT-01, the target adapter",
	spigot_drawing: "Drawing PT-02, the target spigot and its mating pattern",
	standoff_drawing: "Drawing PT-03, the standoff set",
	setup: "The setup: the sensor on its rigid stand and the robot carrying a target on the adapter",
	mount_check: "Cross-section of the mounting stack: flange, adapter, spigot, ball-lock pin, back plate, standoffs, front plate",
	stations: "The nine stations along Z and the series that visit each of them",
	series_z: "Series Z: the step ladder and the ramp, with the approach from below",
};
const DEFAULT_FIGURE_ALT = "Figure";
// Icon of the boxed key message of the three-column slides (columnsWithMessage), by slide id.
const MESSAGE_ICONS = { purpose: "table", prep: "tool", robot_program: "robot", series_cd: "crosshairs" };

// Two drawings side by side ("pair"): the gap between them. Drawings are 3300 x 2100 px; a drawing file that does
// not exist yet is drawn as a placeholder of this pixel size.
const PAIR = {
	gap: GAP, // gap between the two drawings
};
const DRAWING_PX_W = 3300; // pixel size assumed for a placeholder (the size of every procedure drawing)
const DRAWING_PX_H = 2100;

// Labels that are not content from the JSON but are needed to read a diagram.
const EXTRA_LABELS = {
	drawingPending: "drawing pending", // placeholder of a drawing file that does not exist yet, after its part number
	sensor: "Sensor", // bootstrap diagram
	locatingEdges: "Locating edges", // plate drawing legend (the two edges that rest on the adapter's edge pins)
	supportPads: "Support pads", // plate drawing legend (the adapter's three support pads)
};

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

/** Number of lines a text needs when wrapped greedily at the given width (inches) and size (pt). */
function countLines(text, widthIn, pt, bold = false) {
	const charW = (pt * (bold ? CHAR_W_BOLD : CHAR_W_REGULAR)) / 72;
	const maxChars = Math.max(1, Math.floor(widthIn / charW));
	let lines = 1;
	let used = 0;
	for (const word of String(text).split(" ")) {
		const need = used === 0 ? word.length : used + 1 + word.length;
		if (need <= maxChars) {
			used = need;
		} else {
			lines += used === 0 ? 0 : 1;
			used = word.length;
		}
	}
	return lines;
}

/** Estimated height (inches) of a wrapped text block. */
function textHeight(text, widthIn, pt, bold = false) {
	return (countLines(text, widthIn, pt, bold) * pt * BODY_LINE_FACTOR) / 72 + TEXT_SLACK;
}

/** Estimated height of a bulleted list (each item one paragraph). */
function bulletsHeight(items, widthIn, pt) {
	const indentIn = BULLET_INDENT_PT / 72;
	let total = 0;
	for (const item of items) total += textHeight(item, widthIn - indentIn, pt) - TEXT_SLACK + PARA_SPACE_PT / 72;
	return total + TEXT_SLACK;
}

/** Keep numbers and their units together (non-breaking space). */
function glue(str) {
	return String(str).replace(UNIT_PATTERN, "$1" + NBSP + "$2");
}

/** Render a react-icons component to a white (or colored) PNG data URI. */
async function renderIcon(Component, hex) {
	const svg = ReactDOMServer.renderToStaticMarkup(React.createElement(Component, { color: "#" + hex, size: String(ICON_RASTER_PX) }));
	const png = await sharp(Buffer.from(svg)).resize(ICON_RASTER_PX, ICON_RASTER_PX, { fit: "contain", background: { r: 0, g: 0, b: 0, alpha: 0 } }).png().toBuffer();
	return "image/png;base64," + png.toString("base64");
}

// ---------------------------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------------------------
/** Build one deck: read the content JSON, write the .pptx, then put the theme colors into the file. */
async function buildDeck(contentJson, outputPptx) {
	const content = JSON.parse(fs.readFileSync(contentJson, "utf8"));

	const pres = new pptxgen();
	pres.layout = "LAYOUT_WIDE";
	pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
	pres.title = content.slides[0].title; // the title slide's title names the deck
	pres.author = DECK_AUTHOR;
	pres.subject = DECK_SUBJECT;
	const C = pres.SchemeColor; // scheme colors: text1 dk1, text2 dk2, background1 lt1, background2 lt2, accent1..6

	// ---- Icons (white glyphs on colored circles; one teal glyph for list check marks) ----------
	const WHITE_HEX = THEME.colors.lt1;
	const TEAL_HEX = THEME.colors.accent2;
	const iconSources = {
		spray: fa.FaSprayCan,
		ruler: fa.FaRulerHorizontal,
		wrench: fa.FaWrench,
		circle: fa.FaCircle,
		square: fa.FaSquareFull,
		check: fa.FaCheck,
		warning: fa.FaExclamationTriangle,
		move: fa.FaArrowsAlt,
		sliders: fa.FaSlidersH,
		edit: fa.FaEdit,
		tool: fa.FaTools,
		hand: fa.FaHandPaper,
		eye: fa.FaEye,
		swap: fa.FaExchangeAlt,
		clipboard: fa.FaClipboardList,
		clock: fa.FaClock,
		bullseye: fa.FaBullseye,
		list: fa.FaListOl,
		fileCode: fa.FaFileCode,
		fileList: fa.FaListAlt,
		fileAlt: fa.FaFileAlt,
		code: fa.FaCode,
		question: fa.FaQuestion,
		table: fa.FaTable,
		board: fa.FaBorderAll,
		camera: fa.FaCamera,
		input: fa.FaFileImport,
		cogs: fa.FaCogs,
		output: fa.FaFileExport,
		robot: fa.FaRobot,
		clipboardCheck: fa.FaClipboardCheck,
		paint: fa.FaPaintRoller,
		toolbox: fa.FaToolbox,
		gauge: fa.FaTachometerAlt,
		magnet: fa.FaMagnet,
		layers: fa.FaLayerGroup,
		crosshairs: fa.FaCrosshairs,
		layout: fa.FaThLarge, // cell layout (the "layout" icon key of the prep slide)
	};
	const iconWhite = {};
	for (const [key, comp] of Object.entries(iconSources)) iconWhite[key] = await renderIcon(comp, WHITE_HEX);
	const iconTealCheckSquare = await renderIcon(fa.FaCheckSquare, TEAL_HEX);

	// ---- Images: read pixel sizes so aspect ratios are exact ---------------------------------
	const imageInfo = {};
	// Changed from the registration generator: a file that does not exist (a drawing still being made) is not an
	// error; it is recorded as missing, with the pixel size of the procedure drawings and a label made of its part
	// number, and fitImage draws a labeled placeholder box in its slot. A message is printed so that it is not missed.
	async function loadImage(relPath) {
		if (!imageInfo[relPath]) {
			const full = path.join(PRESENTATIONS_DIR, relPath);
			if (!fs.existsSync(full)) {
				const partNumber = path.basename(relPath).split("_")[0]; // "PT-07_T5_cutout_plate.png" -> "PT-07"
				imageInfo[relPath] = { full, w: DRAWING_PX_W, h: DRAWING_PX_H, missing: true, label: `${partNumber}: ${EXTRA_LABELS.drawingPending}` };
				console.warn(`NOTE: ${relPath} is missing; a placeholder "${imageInfo[relPath].label}" is drawn in its slot`);
			} else {
				const meta = await sharp(full).metadata();
				imageInfo[relPath] = { full, w: meta.width, h: meta.height };
			}
		}
		return imageInfo[relPath];
	}
	for (const s of content.slides) {
		if (s.image) await loadImage(s.image);
		if (s.image2) await loadImage(s.image2);
		for (const rel of s.images || []) await loadImage(rel); // the "pair" builder
	}

	// ---- Layouts -----------------------------------------------------------------------------
	pres.defineSlideMaster({
		title: "TITLE_DARK",
		background: { color: C.text2 },
		objects: [
			{ placeholder: { options: { name: "title", type: "title", x: MARGIN, y: MARGIN, w: CONTENT_W, h: DARK_TITLE_H, fontFace: HEAD_FONT_REF, fontSize: DARK_TITLE_PT, bold: true, color: C.background1, align: "left", valign: "bottom", margin: 0 }, text: "Title" } },
			{ placeholder: { options: { name: "subtitle", type: "body", x: MARGIN, y: DARK_SUBTITLE_Y, w: DARK_SUBTITLE_W, h: DARK_SUBTITLE_H, fontSize: DARK_SUBTITLE_PT, color: C.accent6, align: "left", valign: "top", margin: 0 }, text: "Subtitle" } },
			{ placeholder: { options: { name: "footer", type: "body", x: MARGIN, y: DARK_FOOTER_Y, w: CONTENT_W, h: DARK_FOOTER_H, fontSize: FOOTER_PT, color: C.accent6, align: "left", valign: "bottom", margin: 0 }, text: "Footer" } },
		],
	});
	pres.defineSlideMaster({
		title: "TITLE_ONLY",
		background: { color: C.background1 },
		objects: [{ placeholder: { options: { name: "title", type: "title", x: CONTENT_X, y: TITLE_Y, w: CONTENT_W, h: TITLE_H, fontFace: HEAD_FONT_REF, fontSize: TITLE_PT, bold: true, color: C.text2, align: "left", valign: "top", margin: 0 }, text: "Title" } }],
		slideNumber: { x: SLIDE_W - MARGIN - NUMBER_W, y: NUMBER_Y, w: NUMBER_W, h: NUMBER_H, fontSize: NUMBER_PT, color: C.accent5, align: "right", valign: "bottom", margin: 0 },
	});

	// ---- Drawing helpers (bound to this presentation) ----------------------------------------
	const R = pres.ShapeType;

	/** Text box: no padding, top-aligned by default, always a real text box. */
	function text(slide, name, runs, x, y, w, h, o = {}) {
		runs = typeof runs === "string" ? glue(runs) : runs.map((r) => Object.assign({}, r, { text: glue(r.text) }));
		slide.addText(runs, Object.assign({ x, y, w, h, margin: 0, valign: "top", align: "left", color: C.text1, fontSize: BODY_PT, isTextBox: true, objectName: name }, o));
	}

	/** Bulleted list from an array of strings. */
	function bullets(slide, name, items, x, y, w, h, o = {}) {
		const space = o.paraSpacePt === undefined ? PARA_SPACE_PT : o.paraSpacePt;
		const runs = items.map((t, i) => ({ text: t, options: { bullet: { indent: BULLET_INDENT_PT }, breakLine: i < items.length - 1, paraSpaceAfter: space } }));
		const rest = Object.assign({}, o);
		delete rest.paraSpacePt;
		text(slide, name, runs, x, y, w, h, rest);
	}

	/** Card: "tint" (lt2 fill) or "white" (white with a pale outline, used behind figures). */
	function card(slide, name, x, y, w, h, kind = "tint") {
		const opts = { x, y, w, h, rectRadius: CARD_RADIUS, objectName: name };
		if (kind === "tint") {
			opts.fill = { color: C.background2 };
			opts.line = { type: "none" };
		} else if (kind === "dark") {
			opts.fill = { color: C.text1 };
			opts.line = { type: "none" };
		} else {
			opts.fill = { color: C.background1 };
			opts.line = { color: C.accent6, width: OUTLINE_PT };
		}
		slide.addShape(R.roundRect, opts);
	}

	/** Solid circle with a centered number. */
	function numberBadge(slide, name, n, x, y, d, fill, pt = BADGE_NUM_PT) {
		slide.addText(String(n), { x, y, w: d, h: d, shape: R.ellipse, fill: { color: fill }, line: { type: "none" }, color: C.background1, bold: true, fontSize: pt, align: "center", valign: "middle", margin: 0, isTextBox: true, objectName: name });
	}

	/** Solid circle with a centered white icon glyph. */
	function iconBadge(slide, name, iconKey, x, y, d, fill) {
		slide.addShape(R.ellipse, { x, y, w: d, h: d, fill: { color: fill }, line: { type: "none" }, objectName: name + " circle" });
		const g = d * ICON_GLYPH_RATIO;
		slide.addImage({ data: iconWhite[iconKey], x: x + (d - g) / 2, y: y + (d - g) / 2, w: g, h: g, altText: iconKey + " icon", objectName: name + " glyph" });
	}

	/** Place an image inside a box with "contain" semantics (exact aspect ratio). */
	function fitImage(slide, name, rel, bx, by, bw, bh, align = "center") {
		const info = imageInfo[rel];
		const ar = info.w / info.h;
		let w = bw;
		let h = w / ar;
		if (h > bh) {
			h = bh;
			w = h * ar;
		}
		const x = align === "left" ? bx : bx + (bw - w) / 2;
		const y = bottomAligned(align) ? by + bh - h : by + (bh - h) / 2;
		if (info.missing) {
			// Placeholder for a drawing that does not exist yet: a tint box with its label, in the slot the drawing will have
			slide.addText(info.label, { x, y, w, h, shape: R.rect, fill: { color: C.background2 }, line: { color: C.accent5, width: FIGURE_FRAME_PT, dashType: "dash" }, color: C.accent5, fontSize: HEAD_PT, bold: true, align: "center", valign: "middle", margin: 0, isTextBox: true, objectName: name + " placeholder" });
		} else {
			slide.addImage({ path: info.full, x, y, w, h, altText: name, objectName: name });
		}
		return { x, y, w, h };
	}
	function bottomAligned(a) {
		return a === "bottom-left";
	}

	/** A bare figure (no card behind it): the image fitted into its box, with a thin outline of its own size. */
	function framedImage(slide, name, rel, bx, by, bw, bh, align = "center") {
		const box = fitImage(slide, name, rel, bx, by, bw, bh, align);
		slide.addShape(R.rect, { x: box.x, y: box.y, w: box.w, h: box.h, fill: { type: "none" }, line: { color: C.accent6, width: FIGURE_FRAME_PT }, objectName: name + " frame" });
		return box;
	}

	/** Caption in the muted caption style. */
	function caption(slide, name, str, x, y, w, h, o = {}) {
		text(slide, name, str, x, y, w, h, Object.assign({ fontSize: CAPTION_PT, color: C.accent5 }, o));
	}

	/** Stat callout: value above label (stacked) inside a tint card. */
	function statCard(slide, name, stat, x, y, w, h, valuePt, valueH, labelH, labelPt = LABEL_PT) {
		card(slide, name + " card", x, y, w, h);
		// Value and label form one block, centered vertically in the card
		const top = y + (h - valueH - labelH) / 2;
		text(slide, name + " value", stat.value, x + CARD_PAD, top, w - 2 * CARD_PAD, valueH, { fontFace: HEAD_FONT_REF, fontSize: valuePt, bold: true, color: C.accent1, valign: "middle" });
		text(slide, name + " label", stat.label, x + CARD_PAD, top + valueH, w - 2 * CARD_PAD, labelH, { fontSize: labelPt });
	}

	/** Title (placeholder) of a content or dark slide. */
	function title(slide, str) {
		// Changed from the registration generator: a title that the text-width model puts on two lines at TITLE_PT is set a
		// point or two smaller (down to TITLE_MIN_PT) so that it stays on one line, as the layout's title box is one line high.
		let pt = TITLE_PT;
		while (pt > TITLE_MIN_PT && countLines(str, CONTENT_W, pt, true) > 1) pt -= TITLE_STEP_PT;
		slide.addText(pt === TITLE_PT ? str : [{ text: str, options: { fontSize: pt } }], { placeholder: "title" }); // run-level size: the placeholder ignores a size given in its own options
	}

	/**
	 * A vertical list of rows, each with a badge on the left and wrapped text on the right.
	 * Returns the y just below the last row.
	 * opts: x, y, w, pt, badge(i,item)->{kind:'num'|'icon'|'glyph', ...}, badgeD, cardKind (null|'tint'|'dark'|'white'),
	 *       textColor, gap, minRowH, fixedRowH
	 */
	function rowList(slide, name, items, o) {
		let y = o.y;
		const pad = o.cardKind ? o.cardPad : 0;
		const textX = o.x + pad + o.badgeD + GAP_TIGHT;
		const textW = o.w - pad * 2 - o.badgeD - GAP_TIGHT;
		const rowGap = o.gap === undefined ? GAP_TIGHT : o.gap;
		const needH = items.map((item) => Math.max(o.badgeD, textHeight(typeof item === "string" ? item : item.text, textW, o.pt)) + pad * 2);
		// fillH: rows are as tall as their text needs, and share what is left of fillH equally, so the list fills the height
		const extra = o.fillH === undefined ? 0 : (o.fillH - (items.length - 1) * rowGap - needH.reduce((a, b) => a + b, 0)) / items.length;
		if (extra < 0) console.warn(`WARNING: ${name} needs more than ${o.fillH.toFixed(2)} in`);
		items.forEach((item, i) => {
			const str = typeof item === "string" ? item : item.text;
			const rowH = o.fixedRowH || Math.max(o.minRowH || 0, needH[i] + Math.max(0, extra)); // fixedRowH: equal rows, text is known to fit
			if (o.cardKind) card(slide, `${name} row ${i + 1} card`, o.x, y, o.w, rowH, o.cardKind);
			const by = y + (rowH - o.badgeD) / 2;
			const b = o.badge(i, item);
			if (b.kind === "num") numberBadge(slide, `${name} badge ${i + 1}`, b.n, o.x + pad, by, o.badgeD, b.fill, o.badgePt || BADGE_NUM_PT);
			else if (b.kind === "icon") iconBadge(slide, `${name} icon ${i + 1}`, b.icon, o.x + pad, by, o.badgeD, b.fill);
			else slide.addImage({ data: b.data, x: o.x + pad, y: by, w: o.badgeD, h: o.badgeD, altText: "check icon", objectName: `${name} check ${i + 1}` });
			text(slide, `${name} text ${i + 1}`, str, textX, y, textW, rowH, { fontSize: o.pt, color: o.textColor || C.text1, valign: "middle" });
			y += rowH + rowGap;
		});
		return y - rowGap;
	}

	/** Height of a caption of the given width: one line or two, by the text-width model. */
	function captionHeightFor(str, widthIn) {
		return countLines(str, widthIn, CAPTION_PT) === 1 ? CAPTION_LINE_H : CAPTION_TWO_LINE_H;
	}

	/**
	 * Space of a table slide: the caption sits at the bottom of the content area, and the table (and any
	 * visual next to it) fills the rest down to TABLE_CAPTION_GAP above the caption.
	 */
	function tableArea(captionStr) {
		const captionH = captionHeightFor(captionStr, CONTENT_W);
		const captionY = CONTENT_BOTTOM - captionH;
		return { captionY, captionH, tableH: captionY - TABLE_CAPTION_GAP - CONTENT_TOP };
	}

	/**
	 * Table with a dark header row and banded body rows, filling the height h from y. Rows are equally tall
	 * unless a row's text needs more (by the text-width model); a table that cannot fit in h is reported.
	 * o: name, x, y, h, colW[], cellPt, colAlign[], colBold[], colColor[] (scheme colors, default text1)
	 */
	function dataTable(slide, s, o) {
		const [mTop, mRight, mBottom, mLeft] = TABLE_CELL_MARGIN;
		const w = o.colW.reduce((a, b) => a + b, 0);
		const headCell = (str, ci) => ({ text: glue(str), options: { bold: true, color: C.background1, fill: { color: C.text2 }, align: o.colAlign[ci], valign: "middle", fontSize: o.cellPt, margin: TABLE_CELL_MARGIN } });
		const rows = [s.table.header.map(headCell)];
		s.table.rows.forEach((r, i) => {
			const base = { fill: { color: i % 2 === 0 ? C.background1 : C.background2 }, valign: "middle", fontSize: o.cellPt, margin: TABLE_CELL_MARGIN };
			rows.push(r.map((cellText, ci) => ({ text: glue(cellText), options: Object.assign({ align: o.colAlign[ci], bold: o.colBold[ci], color: o.colColor[ci] || C.text1 }, base) })));
		});
		const bodyH = o.h - TABLE_HEAD_H;
		const needHs = s.table.rows.map((r) => {
			const lines = Math.max(...r.map((cellText, ci) => countLines(glue(cellText), o.colW[ci] - mLeft - mRight, o.cellPt, o.colBold[ci])));
			return (lines * o.cellPt * TABLE_ROW_LINE_FACTOR) / 72 + mTop + mBottom;
		});
		// Rows are equally tall; a row whose text needs more than that keeps its own height and the others share the rest
		const evenH = bodyH / s.table.rows.length;
		const tallNeeds = needHs.filter((n) => n > evenH);
		const shortH = (bodyH - tallNeeds.reduce((a, b) => a + b, 0)) / (s.table.rows.length - tallNeeds.length);
		const rowHs = needHs.map((n) => (n > evenH ? n : shortH));
		const totalH = TABLE_HEAD_H + rowHs.reduce((a, b) => a + b, 0);
		// Changed from the registration generator, which compared only the short rows and so missed a table whose rows are all
		// taller than an even share: a table that is taller than its box is reported whatever the rows.
		if (totalH > o.h + TABLE_FIT_TOLERANCE || (tallNeeds.length < s.table.rows.length && shortH < Math.max(...needHs.filter((n) => n <= evenH)) - TABLE_FIT_TOLERANCE)) console.warn(`WARNING: ${o.name} needs ${totalH.toFixed(2)} in and does not fit in ${o.h.toFixed(2)} in`);
		slide.addTable(rows, { x: o.x, y: o.y, w, colW: o.colW, rowH: [TABLE_HEAD_H].concat(rowHs), border: { type: "solid", pt: TABLE_BORDER_PT, color: C.accent6 }, objectName: o.name });
		return { w, h: Math.max(o.h, totalH) };
	}

	/**
	 * A figure at the left (full content height, or IMAGE_BESIDE_MAX_W wide when it is wide) and numbered step
	 * cards stacked to its right; the cards share the content height (rows grow only if their text needs it).
	 * o: figureName
	 */
	function imageWithSteps(slide, s, o) {
		const t = IMAGE_STEPS;
		// Changed again: the figure column is FIGURE_SHARE_WIDE of the content width (or the width that fills the content height,
		// if that is less); the steps wrap in the rest and, when they need more height than the content area has, the gap between
		// them shrinks (down to t.stackGapMin) before anything else. A column that still does not fit is reported; the font stays at stepPt.
		const figW = FIGURE_SHARE_WIDE * CONTENT_W;
		const stepsFit = (gap) => {
			const textW = CONTENT_W - figW - GAP - CARD_PAD - BADGE_D - GAP_TIGHT;
			const need = s.steps.map((st) => Math.max(t.minRowH, textHeight(st.text, textW, t.stepPt) + 2 * t.rowCardPad));
			return { need, gap, extra: (CONTENT_H - (s.steps.length - 1) * gap - need.reduce((a, b) => a + b, 0)) / s.steps.length };
		};
		let fit = stepsFit(t.stackGap);
		while (fit.extra < 0 && fit.gap - ICON_CARD_GAP_STEP >= t.stackGapMin - 1e-9) fit = stepsFit(fit.gap - ICON_CARD_GAP_STEP);
		const img = framedImage(slide, o.figureName, s.image, CONTENT_X, CONTENT_TOP, figW, CONTENT_H, "left");
		const rx = img.x + img.w + GAP;
		const rw = CONTENT_X + CONTENT_W - rx;
		const { need, extra, gap } = fit;
		if (extra < 0) console.warn(`WARNING: ${o.figureName}: steps need ${(-extra * s.steps.length).toFixed(2)} in more than the content height`);
		let y = CONTENT_TOP;
		s.steps.forEach((st, i) => {
			const rowH = need[i] + Math.max(0, extra);
			card(slide, `Step ${st.n} card`, rx, y, rw, rowH);
			numberBadge(slide, `Step ${st.n} badge`, st.n, rx + CARD_PAD / 2, y + (rowH - BADGE_D) / 2, BADGE_D, C.accent1);
			const tx = rx + CARD_PAD / 2 + BADGE_D + GAP_TIGHT;
			text(slide, `Step ${st.n} text`, st.text, tx, y, rx + rw - CARD_PAD / 2 - tx, rowH, { fontSize: t.stepPt, valign: "middle" });
			y += rowH + gap;
		});
	}

	/**
	 * A figure at the left (full content height, or IMAGE_BESIDE_MAX_W wide when it is wide) and check rows to its
	 * right, spread over the full height so the list is centered against the figure.
	 * o: figureName, rowName
	 */
	function imageWithPoints(slide, s, o) {
		const b = IMAGE_POINTS;
		// Changed from the registration generator: the figure is as wide as the rows still fit beside it. Starting from the
		// width that fills the content height (at most b.figureMaxW), the figure narrows in steps of IMAGE_STEP_W, down to
		// IMAGE_BESIDE_MAX_W, until the rows fit the content height at the smallest allowed size (BODY_MIN_PT); the rows then
		// take the largest size of pointPt and its fallbacks at which they fit that width.
		const info = imageInfo[s.image];
		const rowsH = (pt, rowW) => s.points.reduce((sum, str) => sum + Math.max(BADGE_SMALL_D, textHeight(str, rowW - BADGE_SMALL_D - GAP_TIGHT, pt)), 0) + (s.points.length - 1) * b.rowGap;
		const sizes = [b.pointPt].concat(b.pointPtFallbacks);
		const smallest = sizes[sizes.length - 1];
		const rowW = (figW) => CONTENT_W - figW - GAP;
		let figW = Math.min(CONTENT_H * (info.w / info.h), b.figureMaxW);
		while (rowsH(smallest, rowW(figW)) > CONTENT_H && figW - IMAGE_STEP_W >= IMAGE_BESIDE_MAX_W - 1e-9) figW -= IMAGE_STEP_W;
		const pt = sizes.find((size) => rowsH(size, rowW(figW)) <= CONTENT_H) || smallest;
		if (rowsH(pt, rowW(figW)) > CONTENT_H) console.warn(`WARNING: slide "${s.id}": the rows need ${rowsH(pt, rowW(figW)).toFixed(2)} in of ${CONTENT_H.toFixed(2)} in`);
		const img = framedImage(slide, o.figureName, s.image, CONTENT_X, CONTENT_TOP, figW, CONTENT_H, "left");
		const rx = img.x + img.w + GAP;
		const rw = CONTENT_X + CONTENT_W - rx;
		rowList(slide, o.rowName, s.points, { x: rx, y: CONTENT_TOP, w: rw, pt, badgeD: BADGE_SMALL_D, gap: b.rowGap, fillH: CONTENT_H, cardKind: null, cardPad: 0, badge: () => ({ kind: "icon", icon: "check", fill: C.accent2 }) });
	}

	// ---- Slide builders ----------------------------------------------------------------------
	const builders = {};

	builders.title = (slide, s) => {
		title(slide, s.title);
		slide.addText(s.subtitle, { placeholder: "subtitle" });
		slide.addText(s.footer, { placeholder: "footer" });
		// Graphic: the sensor and three boards at three standoffs and tilts, sitting on one baseline (native shapes).
		const a = TITLE_ART;
		const baseline = DARK_FOOTER_Y - a.bottomGap;
		const cy = baseline - a.plateH / 2; // common vertical center of the sensor and the boards
		const sensorX = MARGIN;
		slide.addShape(R.roundRect, { x: sensorX, y: cy - a.sensorH / 2, w: a.sensorW, h: a.sensorH, rectRadius: CARD_RADIUS, fill: { color: C.accent2 }, line: { color: C.accent6, width: a.outlinePt }, objectName: "Graphic sensor body" });
		slide.addShape(R.ellipse, { x: sensorX + a.sensorW - a.lensInset - a.lensD, y: cy - a.lensD / 2, w: a.lensD, h: a.lensD, fill: { color: C.accent3 }, line: { type: "none" }, objectName: "Graphic sensor lens" });
		a.standoffsMm.forEach((mm, i) => {
			const cx = sensorX + a.sensorW + mm * a.inchesPerMm; // board center
			slide.addShape(R.roundRect, { x: cx - a.plateW / 2, y: cy - a.plateH / 2, w: a.plateW, h: a.plateH, rectRadius: a.plateRadius, rotate: a.tiltsDeg[i], fill: { color: C.accent6 }, line: { color: C.accent3, width: a.outlinePt }, objectName: `Graphic board ${i + 1}` });
			slide.addShape(R.ellipse, { x: cx - a.originD / 2, y: cy - a.originD / 2, w: a.originD, h: a.originD, fill: { color: C.accent1 }, line: { type: "none" }, objectName: `Graphic board ${i + 1} tool-frame origin` });
		});
	};

	builders.product = (slide, s) => {
		title(slide, s.title);
		const p = PRODUCT;
		const cardW = (CONTENT_W - 2 * GAP) / s.stats.length;
		const statIcons = ["clipboard", "clock", "bullseye"];
		s.stats.forEach((st, i) => {
			const x = CONTENT_X + i * (cardW + GAP);
			card(slide, `Stat ${i + 1} card`, x, CONTENT_TOP, cardW, p.cardH);
			iconBadge(slide, `Stat ${i + 1} icon`, statIcons[i], x + CARD_PAD, CONTENT_TOP + CARD_PAD, p.badgeD, C.accent2);
			const vy = CONTENT_TOP + CARD_PAD + p.badgeD + GAP_TIGHT / 2;
			text(slide, `Stat ${i + 1} value`, st.value, x + CARD_PAD, vy, cardW - 2 * CARD_PAD, p.valueH, { fontFace: HEAD_FONT_REF, fontSize: STAT_PT, bold: true, color: C.accent1, valign: "middle" });
			text(slide, `Stat ${i + 1} label`, st.label, x + CARD_PAD, vy + p.valueH, cardW - 2 * CARD_PAD, p.labelH, { fontSize: LABEL_PT });
		});
		const my = CONTENT_TOP + p.cardH + GAP;
		iconBadge(slide, "Message icon", "table", CONTENT_X, my + (p.messageH - p.messageBadgeD) / 2, p.messageBadgeD, C.accent1);
		text(slide, "Key message", s.message, CONTENT_X + p.messageBadgeD + GAP_TIGHT, my, CONTENT_W - p.messageBadgeD - GAP_TIGHT, p.messageH, { fontSize: MESSAGE_PT, bold: true, italic: true, color: C.text2, valign: "middle" });
		const by = my + p.messageH + GAP;
		text(slide, "Body", s.body, CONTENT_X, by, CONTENT_W, CONTENT_BOTTOM - by, { fontSize: p.bodyPt });
	};

	builders.flow = (slide, s) => {
		title(slide, s.title);
		const f = FLOW;
		const perRow = Math.ceil(s.steps.length / 2); // seven steps: four in the first row, three in the second
		const cardW = (CONTENT_W - (perRow - 1) * ARROW_GAP) / perRow;
		const cardH = (CONTENT_H - FLOW_ROW_GAP) / 2;
		const pos = (i) => ({ x: CONTENT_X + (i % perRow) * (cardW + ARROW_GAP), y: CONTENT_TOP + Math.floor(i / perRow) * (cardH + FLOW_ROW_GAP) });
		s.steps.forEach((st, i) => {
			const { x, y } = pos(i);
			card(slide, `Step ${st.n} card`, x, y, cardW, cardH);
			numberBadge(slide, `Step ${st.n} badge`, st.n, x + CARD_PAD, y + CARD_PAD, f.numberD, C.accent1);
			// Changed from the registration generator: the head sits beside the number badge, not under it, which gives the text
			// one and a half more lines (the first row's texts run to five lines at 14 pt in a card a quarter of the width)
			const hx = x + CARD_PAD + f.numberD + GAP_TIGHT;
			text(slide, `Step ${st.n} head`, st.head, hx, y + CARD_PAD, x + cardW - CARD_PAD - hx, f.numberD, { fontSize: f.headPt, bold: true, color: C.text2, valign: "middle" });
			const ty = y + CARD_PAD + f.numberD + GAP_TIGHT / 2;
			text(slide, `Step ${st.n} text`, st.text, x + CARD_PAD, ty, cardW - 2 * CARD_PAD, y + cardH - CARD_PAD - ty, { fontSize: f.textPt });
			// Chevron to the next card in the same row (none after the last step, which ends row 2 short)
			if (i % perRow !== perRow - 1 && i < s.steps.length - 1) {
				slide.addShape(R.chevron, { x: x + cardW + (ARROW_GAP - ARROW_W) / 2, y: y + (cardH - ARROW_H) / 2, w: ARROW_W, h: ARROW_H, fill: { color: C.accent2 }, line: { type: "none" }, objectName: `Arrow ${st.n} to ${st.n + 1}` });
			}
		});
		// Return connector from the end of row 1 down and back to the start of row 2
		const last1 = pos(perRow - 1);
		const first2 = pos(perRow);
		const startX = last1.x + cardW / 2;
		const endX = first2.x + cardW / 2;
		const startY = last1.y + cardH;
		const midY = startY + FLOW_ROW_GAP / 2;
		const endY = first2.y - ARROWHEAD_H;
		const line = () => ({ line: { color: C.accent2, width: FLOW_LINE_PT } });
		slide.addShape(R.line, Object.assign({ x: startX, y: startY, w: 0, h: midY - startY, objectName: "Return connector down" }, line()));
		slide.addShape(R.line, Object.assign({ x: endX, y: midY, w: startX - endX, h: 0, objectName: "Return connector across" }, line()));
		slide.addShape(R.line, Object.assign({ x: endX, y: midY, w: 0, h: endY - midY, objectName: "Return connector into row 2" }, line()));
		slide.addShape(R.triangle, { x: endX - ARROWHEAD_W / 2, y: endY, w: ARROWHEAD_W, h: ARROWHEAD_H, flipV: true, fill: { color: C.accent2 }, line: { type: "none" }, objectName: "Return connector arrowhead" });
	};

	builders.fixtures = (slide, s) => {
		title(slide, s.title);
		figureWithCards(slide, s, { sizes: FIXTURES, icons: CARD_ICONS[s.id] || DEFAULT_CARD_ICONS, figureName: FIGURE_ALT[s.id] || DEFAULT_FIGURE_ALT, partName: "Fixture" });
	};

	builders.cost = (slide, s) => {
		title(slide, s.title);
		const c = COST;
		const area = tableArea(s.caption);
		const table = dataTable(slide, s, { name: "Cost table", x: CONTENT_X, y: CONTENT_TOP, h: area.tableH, colW: c.colW, cellPt: c.cellPt, colAlign: ["left", "right"], colBold: [false, true], colColor: [] });
		const rx = CONTENT_X + c.tableW + GAP;
		const rw = CONTENT_W - c.tableW - GAP;
		const cardH = (table.h - (s.stats.length - 1) * STACK_GAP) / s.stats.length; // the stat cards share the table's height
		s.stats.forEach((st, i) => {
			statCard(slide, `Total ${i + 1}`, st, rx, CONTENT_TOP + i * (cardH + STACK_GAP), rw, cardH, c.statValuePt, c.statValueH, c.statLabelH);
		});
		caption(slide, "Cost caption", s.caption, CONTENT_X, area.captionY, CONTENT_W, area.captionH, { valign: "bottom" });
	};

	builders.build_list = (slide, s) => {
		title(slide, s.title);
		const t = BUILD_LIST;
		const area = tableArea(s.caption);
		const table = dataTable(slide, s, { name: "Build list table", x: CONTENT_X, y: CONTENT_TOP, h: area.tableH, colW: t.colW, cellPt: t.cellPt, colAlign: ["left", "left", "left", "left"], colBold: [false, false, false, false], colColor: [] });
		// Visual: a tint card over the table's height with a large teal toolbox circle in its middle
		const vx = CONTENT_X + table.w + GAP;
		const vw = CONTENT_X + CONTENT_W - vx;
		card(slide, "Visual card", vx, CONTENT_TOP, vw, table.h);
		iconBadge(slide, "Toolbox icon", "toolbox", vx + (vw - t.iconD) / 2, CONTENT_TOP + (table.h - t.iconD) / 2, t.iconD, C.accent2);
		caption(slide, "Build list caption", s.caption, CONTENT_X, area.captionY, CONTENT_W, area.captionH, { valign: "bottom" });
	};

	builders.buy_list = (slide, s) => {
		title(slide, s.title);
		const t = BUY_LIST;
		const area = tableArea(s.caption);
		dataTable(slide, s, { name: "Buy list table", x: CONTENT_X, y: CONTENT_TOP, h: area.tableH, colW: t.colW, cellPt: t.cellPt, colAlign: ["left", "left", "right", "left"], colBold: [false, false, true, false], colColor: [] });
		caption(slide, "Buy list caption", s.caption, CONTENT_X, area.captionY, CONTENT_W, area.captionH, { valign: "bottom" });
	};

	builders.plate_spec = (slide, s) => {
		title(slide, s.title);
		const t = PLATE_SPEC;
		const area = tableArea(s.caption);
		const wide = s.table.header.length > 2; // three columns: two target groups, full width, no plate drawing (see PLATE_SPEC)
		const colW = wide ? t.colWWide : t.colW;
		const table = dataTable(slide, s, { name: "Plate specification table", x: CONTENT_X, y: CONTENT_TOP, h: area.tableH, colW, cellPt: wide ? t.cellWidePt : t.cellPt, colAlign: colW.map(() => "left"), colBold: colW.map((_, i) => i === 0), colColor: [C.text2] });
		if (wide) {
			caption(slide, "Plate specification caption", s.caption, CONTENT_X, area.captionY, CONTENT_W, area.captionH, { valign: "bottom" });
			return;
		}
		// Visual: the plate drawn to scale in a white card, its two locating edges in orange, the three support pads as circles
		const vx = CONTENT_X + table.w + GAP;
		const vw = CONTENT_X + CONTENT_W - vx;
		card(slide, "Plate drawing card", vx, CONTENT_TOP, vw, table.h, "white");
		const pw = t.plateWmm * t.scale;
		const ph = t.plateHmm * t.scale;
		const legendH = 2 * t.legendH + t.legendGap;
		const blockH = ph + t.legendGap + legendH;
		const px = vx + (vw - pw) / 2;
		const py = CONTENT_TOP + (table.h - blockH) / 2;
		slide.addShape(R.rect, { x: px, y: py, w: pw, h: ph, fill: { color: C.background2 }, line: { color: C.accent5, width: t.outlinePt }, objectName: "Plate outline" });
		// Locating edges (bottom long edge and left short edge), drawn on top of the outline
		slide.addShape(R.line, { x: px, y: py + ph, w: pw, h: 0, line: { color: C.accent1, width: t.edgePt }, objectName: "Plate bottom edge (locating)" });
		slide.addShape(R.line, { x: px, y: py, w: 0, h: ph, line: { color: C.accent1, width: t.edgePt }, objectName: "Plate left edge (locating)" });
		// Three support pads at drawing SC1-05 positions: two near the bottom corners, one at the top middle
		const padD = t.padDmm * t.scale; // pad diameter on the slide (in)
		// Convert each pad center from mm about the plate center (y up) to slide inches (y down).
		const pads = t.padsMm.map(([xmm, ymm]) => [px + pw / 2 + xmm * t.scale, py + ph / 2 - ymm * t.scale]);
		pads.forEach(([cx, cy], i) => {
			slide.addShape(R.ellipse, { x: cx - padD / 2, y: cy - padD / 2, w: padD, h: padD, fill: { color: C.accent2 }, line: { type: "none" }, objectName: `Support pad ${i + 1}` });
		});
		// Legend under the drawing: key on the left, label on the right, as one block centered in the card
		const legend = [
			{ label: EXTRA_LABELS.locatingEdges, key: (kx, ky) => slide.addShape(R.line, { x: kx, y: ky, w: t.legendKeyW, h: 0, line: { color: C.accent1, width: t.edgePt }, objectName: "Legend key locating edges" }) },
			{ label: EXTRA_LABELS.supportPads, key: (kx, ky) => slide.addShape(R.ellipse, { x: kx + (t.legendKeyW - t.padDmm * t.scale) / 2, y: ky - t.padDmm * t.scale / 2, w: t.padDmm * t.scale, h: t.padDmm * t.scale, fill: { color: C.accent2 }, line: { type: "none" }, objectName: "Legend key support pads" }) },
		];
		const ly0 = py + ph + t.legendGap;
		legend.forEach((it, i) => {
			const ly = ly0 + i * (t.legendH + t.legendGap);
			it.key(px, ly + t.legendH / 2);
			text(slide, `Legend ${it.label}`, it.label, px + t.legendKeyW + GAP_TIGHT, ly, pw - t.legendKeyW - GAP_TIGHT, t.legendH, { fontSize: t.legendPt, valign: "middle" });
		});
		caption(slide, "Plate specification caption", s.caption, CONTENT_X, area.captionY, CONTENT_W, area.captionH, { valign: "bottom" });
	};

	/**
	 * A white card at the left holding a wide figure and its caption (centered vertically as a group), and icon
	 * cards stacked to its right. Card heights follow the text they hold (head line plus wrapped text), scaled
	 * together to fill the column; a column whose text needs more than the height available is reported.
	 * o: sizes (stackGap, stackGapMin, iconD, headPt, textPt, cardPad, textRightPad, stackGap), icons[], figureName, partName
	 */
	function figureWithCards(slide, s, o) {
		const d = o.sizes;
		// Changed again: the figure column takes FIGURE_SHARE_WIDE of the content width (the registration generator, and the first
		// version of this one, narrowed it to fit the text). The icon cards wrap their text in what is left; when they need more
		// height than the column has, the gap between them shrinks (down to d.stackGapMin) before anything else; a column that
		// still does not fit is reported, and the font is never taken below d.textPt.
		const figureCardW = FIGURE_SHARE_WIDE * CONTENT_W;
		const cardsNeed = (stackGap) => {
			const rx = CONTENT_X + figureCardW + GAP;
			const tw = CONTENT_X + CONTENT_W - d.sidePad - d.textRightPad - (rx + d.sidePad + d.iconD + d.sidePad);
			const needH = s.cards.map((c) => 2 * d.cardPad + (d.headPt * BODY_LINE_FACTOR) / 72 + textHeight(c.text, tw, d.textPt));
			return { needH, tw, stackGap, scale: (CONTENT_H - (s.cards.length - 1) * stackGap) / needH.reduce((x, y) => x + y, 0) };
		};
		let fit = cardsNeed(d.stackGap);
		while (fit.scale < 1 && fit.stackGap - ICON_CARD_GAP_STEP >= d.stackGapMin - 1e-9) fit = cardsNeed(fit.stackGap - ICON_CARD_GAP_STEP);
		const innerW = figureCardW - 2 * CARD_PAD;
		card(slide, "Figure card", CONTENT_X, CONTENT_TOP, figureCardW, CONTENT_H, "white");
		const info = imageInfo[s.image];
		const imgH = innerW * (info.h / info.w);
		const captionH = textHeight(s.caption, innerW, CAPTION_PT);
		const gy = CONTENT_TOP + (CONTENT_H - (imgH + CAPTION_GAP + captionH)) / 2;
		fitImage(slide, o.figureName, s.image, CONTENT_X + CARD_PAD, gy, innerW, imgH);
		caption(slide, "Figure caption", s.caption, CONTENT_X + CARD_PAD, gy + imgH + CAPTION_GAP, innerW, captionH);
		const rx = CONTENT_X + figureCardW + GAP;
		const rw = CONTENT_X + CONTENT_W - rx;
		const tx = rx + d.sidePad + d.iconD + d.sidePad;
		const { needH, tw, scale, stackGap } = fit;
		if (scale < 1) console.warn(`WARNING: slide "${s.id}": ${o.partName} cards need ${(1 / scale).toFixed(2)} times the height available`);
		let y = CONTENT_TOP;
		s.cards.forEach((c, i) => {
			const cardH = needH[i] * scale;
			card(slide, `${o.partName} ${i + 1} card`, rx, y, rw, cardH);
			iconBadge(slide, `${o.partName} ${i + 1} icon`, o.icons[i], rx + d.sidePad, y + (cardH - d.iconD) / 2, d.iconD, C.accent2);
			text(slide, `${o.partName} ${i + 1} text`, [{ text: c.head, options: { fontSize: d.headPt, bold: true, color: C.text2, breakLine: true } }, { text: c.text, options: { fontSize: d.textPt } }], tx, y + d.cardPad, tw, cardH - 2 * d.cardPad, { valign: "middle" });
			y += cardH + stackGap;
		});
	}

	builders.board_build = (slide, s) => {
		title(slide, s.title);
		figureWithCards(slide, s, { sizes: BOARD_BUILD, icons: ["board", "paint", "ruler", "layers"], figureName: "Front face of the board and its adapter", partName: "Board part" });
	};

	builders.runout = (slide, s) => {
		title(slide, s.title);
		figureWithCards(slide, s, { sizes: RUNOUT, icons: CARD_ICONS[s.id] || DEFAULT_CARD_ICONS, figureName: FIGURE_ALT[s.id] || DEFAULT_FIGURE_ALT, partName: "Run-out part" });
	};

	/**
	 * Two drawings side by side at one height, each as large as the content area allows, the caption under them.
	 * New in this generator (edge_target_drawings, feature_plate_drawings). The height is the largest at which both
	 * fit: limited by the content height less the caption, or by the content width shared by the two aspect ratios.
	 * The drawings and the caption form one group, centered vertically in the content area. A drawing file that does
	 * not exist yet is drawn as a labeled placeholder of the same size (see loadImage).
	 */
	builders.pair = (slide, s) => {
		title(slide, s.title);
		const p = PAIR;
		const n = s.images.length;
		const infos = s.images.map((rel) => imageInfo[rel]);
		const ratioSum = infos.reduce((sum, info) => sum + info.w / info.h, 0);
		const captionH = captionHeightFor(s.caption, CONTENT_W);
		const maxH = CONTENT_H - CAPTION_GAP - captionH;
		const imgH = Math.min(maxH, (CONTENT_W - (n - 1) * p.gap) / ratioSum);
		const groupW = imgH * ratioSum + (n - 1) * p.gap;
		const gx = CONTENT_X + (CONTENT_W - groupW) / 2;
		const gy = CONTENT_TOP + (CONTENT_H - (imgH + CAPTION_GAP + captionH)) / 2;
		let x = gx;
		s.images.forEach((rel, i) => {
			const w = imgH * (infos[i].w / infos[i].h);
			framedImage(slide, `Drawing ${i + 1}: ${path.basename(rel, ".png")}`, rel, x, gy, w, imgH, "left");
			x += w + p.gap;
		});
		caption(slide, "Figure caption", s.caption, gx, gy + imgH + CAPTION_GAP, groupW, captionH);
	};

	builders.suppliers = (slide, s) => {
		title(slide, s.title);
		const t = SUPPLIERS;
		const area = tableArea(s.caption);
		dataTable(slide, s, { name: "Suppliers table", x: CONTENT_X, y: CONTENT_TOP, h: area.tableH, colW: t.colW, cellPt: t.cellPt, colAlign: ["left", "left", "left"], colBold: [true, false, false], colColor: [C.text2] });
		caption(slide, "Suppliers caption", s.caption, CONTENT_X, area.captionY, CONTENT_W, area.captionH, { valign: "bottom" });
	};

	builders.acceptance = (slide, s) => {
		title(slide, s.title);
		const a = ACCEPTANCE;
		// Equal rows over the full height: the longest item wraps to two lines, which a row holds
		const rowH = (CONTENT_H - (s.checklist.length - 1) * a.rowGap) / s.checklist.length;
		rowList(slide, "Checklist", s.checklist, { x: CONTENT_X, y: CONTENT_TOP, w: a.listW, pt: a.textPt, badgeD: a.iconD, gap: a.rowGap, fixedRowH: rowH, cardKind: null, cardPad: 0, badge: () => ({ kind: "glyph", data: iconTealCheckSquare }) });
		const vx = CONTENT_X + a.listW + GAP;
		const vw = CONTENT_X + CONTENT_W - vx;
		card(slide, "Visual card", vx, CONTENT_TOP, vw, CONTENT_H);
		iconBadge(slide, "Clipboard check icon", "clipboardCheck", vx + (vw - a.badgeD) / 2, CONTENT_TOP + (CONTENT_H - a.badgeD) / 2, a.badgeD, C.accent2);
	};

	builders.approach = (slide, s) => {
		title(slide, s.title);
		imageWithSteps(slide, s, { figureName: FIGURE_ALT[s.id] || DEFAULT_FIGURE_ALT });
	};

	builders.board = (slide, s) => {
		title(slide, s.title);
		imageWithPoints(slide, s, { figureName: FIGURE_ALT[s.id] || DEFAULT_FIGURE_ALT, rowName: "Board point" });
	};

	builders.residuals = (slide, s) => {
		title(slide, s.title);
		imageWithPoints(slide, s, { figureName: "The normal residual and the offset residual between the predicted and the measured plane", rowName: "Residual point" });
	};

	builders.bootstrap = (slide, s) => {
		title(slide, s.title);
		const b = BOOTSTRAP;
		const cols = b.columns;
		const rows = Math.ceil(s.boot.length / cols);
		const gridH = rows * b.frameH + (rows - 1) * b.rowGap;
		// Sensor rectangle on the left, drawn the same height as the frame grid
		const sensorX = CONTENT_X;
		const sensorY = CONTENT_TOP;
		slide.addShape(R.roundRect, { x: sensorX, y: sensorY, w: b.sensorW, h: gridH, rectRadius: CARD_RADIUS, fill: { color: C.text2 }, line: { type: "none" }, objectName: "Sensor body" });
		slide.addShape(R.ellipse, { x: sensorX + (b.sensorW - b.lensD) / 2, y: sensorY + CARD_PAD, w: b.lensD, h: b.lensD, fill: { color: C.accent3 }, line: { type: "none" }, objectName: "Sensor lens" });
		text(slide, "Sensor label", EXTRA_LABELS.sensor, sensorX, sensorY + CARD_PAD + b.lensD + GAP_TIGHT / 2, b.sensorW, gridH - (CARD_PAD + b.lensD + GAP_TIGHT / 2), { fontSize: b.sensorPt, bold: true, color: C.background1, align: "center", valign: "top" });
		// Frames, one per bootstrap capture: what the sensor sees, with its text to the right
		const framesX = sensorX + b.sensorW + ARROW_GAP;
		const cellW = (CONTENT_X + CONTENT_W - framesX - (cols - 1) * GAP) / cols;
		slide.addShape(R.chevron, { x: sensorX + b.sensorW + (ARROW_GAP - ARROW_W) / 2, y: sensorY + (gridH - ARROW_H) / 2, w: ARROW_W, h: ARROW_H, fill: { color: C.accent2 }, line: { type: "none" }, objectName: "Sensor view arrow" });
		// Size of each circle (w x h): facing the sensor, then tilted (top, bottom: narrower in height; left, right: in width), then farther away
		const placement = [
			{ w: b.bigD, h: b.bigD }, // boot01: facing, near the center
			{ w: b.bigD, h: b.bigD * b.tiltRatio }, // boot02: top edge toward the sensor
			{ w: b.bigD, h: b.bigD * b.tiltRatio }, // boot03: bottom edge toward the sensor
			{ w: b.bigD * b.tiltRatio, h: b.bigD }, // boot04: left edge toward the sensor
			{ w: b.bigD * b.tiltRatio, h: b.bigD }, // boot05: right edge toward the sensor
			{ w: b.farD, h: b.farD }, // boot06: farther, so drawn smaller
		];
		s.boot.forEach((bt, i) => {
			const fx = framesX + (i % cols) * (cellW + GAP);
			const fy = sensorY + Math.floor(i / cols) * (b.frameH + b.rowGap);
			card(slide, `Frame ${bt.id}`, fx, fy, b.frameW, b.frameH, "white");
			const cx = fx + b.frameW / 2;
			const cy = fy + b.frameH / 2;
			const cross = () => ({ line: { color: C.accent6, width: b.crossPt } });
			slide.addShape(R.line, Object.assign({ x: fx + b.frameLineGap, y: cy, w: b.frameW - 2 * b.frameLineGap, h: 0, objectName: `Frame ${bt.id} crosshair horizontal` }, cross()));
			slide.addShape(R.line, Object.assign({ x: cx, y: fy + b.frameLineGap, w: 0, h: b.frameH - 2 * b.frameLineGap, objectName: `Frame ${bt.id} crosshair vertical` }, cross()));
			const p = placement[i];
			slide.addText(bt.id, { x: cx - p.w / 2, y: cy - p.h / 2, w: p.w, h: p.h, shape: R.ellipse, fill: { color: C.accent1 }, line: { type: "none" }, color: C.background1, bold: true, fontSize: DIAGRAM_LABEL_PT, align: "center", valign: "middle", margin: 0, wrap: false, isTextBox: true, objectName: `Circle ${bt.id}` });
			text(slide, `Text ${bt.id}`, bt.text, fx + b.frameW + b.textGap, fy, cellW - b.frameW - b.textGap, b.frameH, { fontSize: b.textPt, valign: "middle" });
		});
		const py = sensorY + gridH + GAP;
		card(slide, "Points card", CONTENT_X, py, CONTENT_W, CONTENT_BOTTOM - py);
		bullets(slide, "Bootstrap points", s.points, CONTENT_X + CARD_PAD, py + GAP_TIGHT, CONTENT_W - 2 * CARD_PAD, CONTENT_BOTTOM - py - 2 * GAP_TIGHT, { fontSize: b.pointsPt, valign: "middle" });
	};

	builders.plan = (slide, s) => {
		title(slide, s.title);
		const p = PLAN;
		// Figure as wide as the target allows, but never taller than the space above its caption
		const info = imageInfo[s.image];
		const maxH = CONTENT_H - CAPTION_GAP - p.captionH;
		const imgW = Math.min(p.imageTargetW, maxH * (info.w / info.h));
		const img = framedImage(slide, "Planned poses: side view and front view", s.image, CONTENT_X, CONTENT_TOP, imgW, maxH, "left");
		caption(slide, "Figure caption", s.caption, CONTENT_X, img.y + img.h + CAPTION_GAP, img.w, p.captionH);
		// Stat callouts stacked in the narrower right column, over the full content height
		const rx = CONTENT_X + img.w + GAP;
		const rw = CONTENT_X + CONTENT_W - rx;
		const statH = (CONTENT_H - (s.stats.length - 1) * STACK_GAP) / s.stats.length;
		s.stats.forEach((st, i) => {
			const y = CONTENT_TOP + i * (statH + STACK_GAP);
			card(slide, `Stat ${i + 1} card`, rx, y, rw, statH);
			const labelH = textHeight(st.label, rw - 2 * p.cardPad, p.labelPt);
			const top = y + (statH - p.valueH - labelH) / 2; // value and label form one block, centered in the card
			text(slide, `Stat ${i + 1} value`, st.value, rx + p.cardPad, top, rw - 2 * p.cardPad, p.valueH, { fontFace: HEAD_FONT_REF, fontSize: p.valuePt, bold: true, color: C.accent1, valign: "middle" });
			text(slide, `Stat ${i + 1} label`, st.label, rx + p.cardPad, top + p.valueH, rw - 2 * p.cardPad, labelH, { fontSize: p.labelPt });
		});
	};

	builders.loop = (slide, s) => {
		title(slide, s.title);
		const l = LOOP;
		const n = s.steps.length;
		const cardW = (CONTENT_W - (n - 1) * ARROW_GAP) / n;
		s.steps.forEach((st, i) => {
			const x = CONTENT_X + i * (cardW + ARROW_GAP);
			card(slide, `Step ${st.n} card`, x, CONTENT_TOP, cardW, l.cardH);
			numberBadge(slide, `Step ${st.n} badge`, st.n, x + CARD_PAD, CONTENT_TOP + CARD_PAD, BADGE_D, C.accent1);
			// Changed from the registration generator: the head sits beside the number badge, as on the flow slide
			const hx = x + CARD_PAD + BADGE_D + GAP_TIGHT;
			text(slide, `Step ${st.n} head`, st.head, hx, CONTENT_TOP + CARD_PAD, x + cardW - CARD_PAD - hx, BADGE_D, { fontSize: l.headPt, bold: true, color: C.text2, valign: "middle" });
			const ty = CONTENT_TOP + CARD_PAD + BADGE_D + GAP_TIGHT / 2;
			text(slide, `Step ${st.n} text`, st.text, x + CARD_PAD, ty, cardW - 2 * CARD_PAD, CONTENT_TOP + l.cardH - CARD_PAD - ty, { fontSize: l.textPt });
			if (i < n - 1) slide.addShape(R.chevron, { x: x + cardW + (ARROW_GAP - ARROW_W) / 2, y: CONTENT_TOP + (l.cardH - ARROW_H) / 2, w: ARROW_W, h: ARROW_H, fill: { color: C.accent2 }, line: { type: "none" }, objectName: `Arrow ${st.n} to ${st.n + 1}` });
		});
		const ry = CONTENT_TOP + l.cardH + GAP;
		const rowH = CONTENT_BOTTOM - ry;
		s.stats.forEach((st, i) => {
			statCard(slide, `Stat ${i + 1}`, st, CONTENT_X + i * (cardW + GAP), ry, cardW, rowH, l.statValuePt, l.statValueH, l.statLabelH);
		});
		const mx = CONTENT_X + s.stats.length * (cardW + GAP);
		const mw = CONTENT_X + CONTENT_W - mx;
		slide.addShape(R.roundRect, { x: mx, y: ry, w: mw, h: rowH, rectRadius: CARD_RADIUS, fill: { color: C.background2 }, line: { color: C.accent1, width: OUTLINE_PT * 2 }, objectName: "Message box" });
		iconBadge(slide, "Message icon", "list", mx + CARD_PAD, ry + (rowH - l.messageBadgeD) / 2, l.messageBadgeD, C.accent1);
		const mtx = mx + CARD_PAD + l.messageBadgeD + GAP_TIGHT;
		text(slide, "Key message", s.message, mtx, ry, mx + mw - CARD_PAD - mtx, rowH, { fontSize: BOX_MESSAGE_PT, bold: true, color: C.text2, valign: "middle" });
	};

	builders.manifest = (slide, s) => {
		title(slide, s.title);
		const m = MANIFEST;
		const n = s.columns.length;
		const colW = (CONTENT_W - (n - 1) * GAP) / n;
		const colsH = CONTENT_H - GAP - m.exampleBoxH;
		const icons = ["fileCode", "fileList"];
		s.columns.forEach((col, i) => {
			const x = CONTENT_X + i * (colW + GAP);
			card(slide, `Option ${i + 1} card`, x, CONTENT_TOP, colW, colsH);
			// Filled header block inset inside the card
			const hx = x + GAP_TIGHT / 2;
			const hy = CONTENT_TOP + GAP_TIGHT / 2;
			const hw = colW - GAP_TIGHT;
			slide.addShape(R.roundRect, { x: hx, y: hy, w: hw, h: m.headBarH, rectRadius: CARD_RADIUS, fill: { color: C.text2 }, line: { type: "none" }, objectName: `Option ${i + 1} header block` });
			iconBadge(slide, `Option ${i + 1} icon`, icons[i], hx + GAP_TIGHT / 2, hy + (m.headBarH - BADGE_SMALL_D) / 2, BADGE_SMALL_D, C.accent1);
			const tx = hx + GAP_TIGHT / 2 + BADGE_SMALL_D + GAP_TIGHT / 2;
			text(slide, `Option ${i + 1} head`, col.head, tx, hy, hx + hw - GAP_TIGHT / 2 - tx, m.headBarH, { fontFace: HEAD_FONT_REF, fontSize: m.headPt, bold: true, color: C.background1, valign: "middle" });
			const by = hy + m.headBarH + GAP_TIGHT;
			bullets(slide, `Option ${i + 1} points`, col.points, x + CARD_PAD, by, colW - 2 * CARD_PAD, CONTENT_TOP + colsH - CARD_PAD - by, { fontSize: m.bulletPt, paraSpacePt: m.paraSpacePt });
		});
		const ey = CONTENT_TOP + colsH + GAP;
		slide.addShape(R.roundRect, { x: CONTENT_X, y: ey, w: CONTENT_W, h: m.exampleBoxH, rectRadius: CARD_RADIUS, fill: { color: C.background2 }, line: { color: C.accent6, width: OUTLINE_PT }, objectName: "Example box" });
		iconBadge(slide, "Example icon", "fileAlt", CONTENT_X + GAP_TIGHT, ey + (m.exampleBoxH - m.exampleBadgeD) / 2, m.exampleBadgeD, C.accent2);
		const ex = CONTENT_X + GAP_TIGHT + m.exampleBadgeD + GAP_TIGHT;
		text(slide, "Pose-log example line", s.example, ex, ey, CONTENT_X + CONTENT_W - GAP_TIGHT - ex, m.exampleBoxH, { fontFace: MONO_FONT, fontSize: MONO_PT, color: C.text1, valign: "middle" });
	};

	/**
	 * Cards in a grid with a head, a flag line and text each. Changed from the registration generator, which had a fixed
	 * 2 x 2 grid: the cards per row come from CHECKS.perRow by card count (five cards: three, then two), the card
	 * heights follow their text (each row as tall as its tallest card needs, the spare height shared equally),
	 * the caption height follows its length, and the icon and its color come from CHECKS_ICONS by slide id.
	 */
	builders.checks = (slide, s) => {
		title(slide, s.title);
		const k = CHECKS;
		const perRow = k.perRow[s.cards.length] || [Math.ceil(s.cards.length / 2), Math.floor(s.cards.length / 2)];
		const icons = CHECKS_ICONS[s.id] || { icons: [], fill: "accent4" };
		const captionH = captionHeightFor(s.caption, CONTENT_W);
		const captionY = CONTENT_BOTTOM - captionH;
		const gridH = captionY - k.gap - CONTENT_TOP;
		// Positions: row r holds perRow[r] cards of equal width
		const slots = [];
		let idx = 0;
		perRow.forEach((n, r) => {
			const cardW = (CONTENT_W - (n - 1) * GAP) / n;
			for (let c = 0; c < n && idx < s.cards.length; c++, idx++) slots.push({ row: r, x: CONTENT_X + c * (cardW + GAP), cardW });
		});
		// Row heights: the text each row's tallest card needs (head, flag, text), plus the padding, plus an equal share of what is left
		const textW = (slot) => slot.cardW - 2 * k.cardPad - k.iconD - GAP_TIGHT;
		const needH = s.cards.map((c, i) => Math.max(k.iconD, (k.headPt * BODY_LINE_FACTOR) / 72 * countLines(c.head, textW(slots[i]), k.headPt, true) + (k.flagPt * BODY_LINE_FACTOR) / 72 * countLines(c.flag, textW(slots[i]), k.flagPt, true) + textHeight(c.text, textW(slots[i]), k.textPt) + PARA_SPACE_PT / 72) + 2 * k.cardPad);
		const rowNeed = perRow.map((_, r) => Math.max(...needH.filter((_, i) => slots[i].row === r)));
		const extra = (gridH - (perRow.length - 1) * k.gap - rowNeed.reduce((a, b) => a + b, 0)) / perRow.length;
		if (extra * perRow.length < -TABLE_FIT_TOLERANCE) console.warn(`WARNING: slide "${s.id}": the cards need ${(-extra * perRow.length).toFixed(2)} in more than the grid has`);
		const rowH = rowNeed.map((n) => n + Math.max(0, extra));
		const rowY = rowH.map((_, r) => CONTENT_TOP + rowH.slice(0, r).reduce((a, b) => a + b, 0) + r * k.gap);
		s.cards.forEach((c, i) => {
			const { row, x, cardW } = slots[i];
			const y = rowY[row];
			const cardH = rowH[row];
			card(slide, `Check ${i + 1} card`, x, y, cardW, cardH, "white");
			iconBadge(slide, `Check ${i + 1} icon`, icons.icons[i] || "warning", x + k.cardPad, y + k.cardPad, k.iconD, C[icons.fill]);
			const tx = x + k.cardPad + k.iconD + GAP_TIGHT;
			const tw = x + cardW - k.cardPad - tx;
			text(
				slide,
				`Check ${i + 1} text`,
				[
					{ text: c.head, options: { fontSize: k.headPt, bold: true, color: C.text2, breakLine: true, paraSpaceAfter: PARA_SPACE_PT / 2 } },
					{ text: c.flag, options: { fontSize: k.flagPt, bold: true, color: C.accent1, breakLine: true, paraSpaceAfter: PARA_SPACE_PT / 2 } },
					{ text: c.text, options: { fontSize: k.textPt } },
				],
				tx,
				y + k.cardPad,
				tw,
				cardH - 2 * k.cardPad,
				{ valign: "top" }
			);
		});
		text(slide, "Checks caption", s.caption, CONTENT_X, captionY, CONTENT_W, captionH, { fontSize: CAPTION_PT, color: C.accent5, valign: "bottom" });
	};

	/**
	 * Columns of (teal icon circle, head, bullets) over one boxed key message.
	 * o: { sizes, bulletPt, boxFill, boxLine, boxIcon, boxIconFill, textPt }
	 */
	function columnsWithMessage(slide, s, o) {
		const d = o.sizes;
		const n = s.columns.length;
		const colW = (CONTENT_W - (n - 1) * GAP) / n;
		const colsH = CONTENT_H - d.messageGap - o.messageH;
		s.columns.forEach((col, i) => {
			const x = CONTENT_X + i * (colW + GAP);
			card(slide, `Column ${i + 1} card`, x, CONTENT_TOP, colW, colsH);
			iconBadge(slide, `Column ${i + 1} icon`, col.icon, x + CARD_PAD, CONTENT_TOP + CARD_PAD + (d.headH - d.iconD) / 2, d.iconD, C.accent2);
			text(slide, `Column ${i + 1} head`, col.head, x + CARD_PAD + d.iconD + GAP_TIGHT, CONTENT_TOP + CARD_PAD, colW - 2 * CARD_PAD - d.iconD - GAP_TIGHT, d.headH, { fontFace: HEAD_FONT_REF, fontSize: d.headPt, bold: true, color: C.text2, valign: "middle" });
			const by = CONTENT_TOP + CARD_PAD + d.headH + d.bulletGap;
			bullets(slide, `Column ${i + 1} points`, col.points, x + CARD_PAD, by, colW - 2 * CARD_PAD, CONTENT_TOP + colsH - d.bottomPad - by, { fontSize: o.bulletPt, paraSpacePt: o.paraSpacePt });
		});
		const my = CONTENT_TOP + colsH + d.messageGap;
		slide.addShape(R.roundRect, { x: CONTENT_X, y: my, w: CONTENT_W, h: o.messageH, rectRadius: CARD_RADIUS, fill: { color: o.boxFill }, line: { color: o.boxLine, width: OUTLINE_PT * 2 }, objectName: "Message box" });
		iconBadge(slide, "Message icon", o.boxIcon, CONTENT_X + CARD_PAD, my + (o.messageH - d.messageIconD) / 2, d.messageIconD, o.boxIconFill);
		const mtx = CONTENT_X + CARD_PAD + d.messageIconD + GAP_TIGHT;
		text(slide, "Key message", s.message, mtx, my, CONTENT_X + CONTENT_W - CARD_PAD - mtx, o.messageH, { fontSize: BOX_MESSAGE_PT, bold: true, color: C.text2, valign: "middle" });
	}

	builders.robot_program = (slide, s) => {
		title(slide, s.title);
		const r = ROBOT_PROGRAM;
		// Like the loop slide's message box: tint fill, orange outline, orange icon circle
		columnsWithMessage(slide, s, { sizes: COLUMNS_MESSAGE, bulletPt: r.bulletPt, paraSpacePt: r.paraSpacePt, messageH: r.messageH, boxFill: C.background2, boxLine: C.accent1, boxIcon: MESSAGE_ICONS[s.id] || "robot", boxIconFill: C.accent1 });
	};

	// The orienting slide (what is tested and why) uses the same three-column-plus-message layout as prep.
	builders.purpose = (slide, s) => builders.prep(slide, s);

	builders.prep = (slide, s) => {
		title(slide, s.title);
		const r = PREP;
		// Same helper and box as robot_program: tint fill, orange outline, orange icon circle
		columnsWithMessage(slide, s, { sizes: COLUMNS_MESSAGE, bulletPt: r.bulletPt, paraSpacePt: r.paraSpacePt, messageH: r.messageH, boxFill: C.background2, boxLine: C.accent1, boxIcon: MESSAGE_ICONS[s.id] || "tool", boxIconFill: C.accent1 });
	};

	builders.spoilers = (slide, s) => {
		title(slide, s.title);
		const sp = SPOILERS;
		const rowH = (CONTENT_H - (sp.rowsFirstColumn - 1) * sp.rowGap) / sp.rowsFirstColumn;
		const colW = (CONTENT_W - (sp.columns - 1) * GAP) / sp.columns;
		const left = s.items.slice(0, sp.rowsFirstColumn);
		const right = s.items.slice(sp.rowsFirstColumn);
		[left, right].forEach((items, ci) => {
			rowList(slide, `Spoiler column ${ci + 1}`, items, {
				x: CONTENT_X + ci * (colW + GAP),
				y: CONTENT_TOP,
				w: colW,
				pt: sp.textPt,
				badgeD: sp.iconD,
				gap: sp.rowGap,
				// The first column's rows are as tall as an equal share of the height; the shorter second column's rows share the whole
				// content height, so both columns end together (changed from the registration generator, where the shorter column ended early)
				...(ci === 0 ? { minRowH: rowH } : { fillH: CONTENT_H }),
				cardKind: "tint",
				cardPad: sp.cardPad,
				badge: (i, item) => ({ kind: "icon", icon: item.icon, fill: C.accent4 }),
			});
		});
	};

	builders.deliverables = (slide, s) => {
		title(slide, s.title);
		const d = DELIVERABLES;
		const listW = d.listW;
		rowList(slide, "Checklist", s.checklist, { x: CONTENT_X, y: CONTENT_TOP, w: listW, pt: d.textPt, badgeD: d.iconD, gap: d.rowGap, fillH: CONTENT_H, cardKind: null, cardPad: 0, badge: () => ({ kind: "glyph", data: iconTealCheckSquare }) });
		const sx = CONTENT_X + listW + GAP;
		const sw = CONTENT_W - listW - GAP;
		card(slide, "Software card", sx, CONTENT_TOP, sw, CONTENT_H);
		iconBadge(slide, "Software icon", "code", sx + CARD_PAD, CONTENT_TOP + CARD_PAD, d.softwareIconD, C.accent2);
		text(slide, "Software head", s.software.head, sx + CARD_PAD + d.softwareIconD + GAP_TIGHT, CONTENT_TOP + CARD_PAD, sw - 2 * CARD_PAD - d.softwareIconD - GAP_TIGHT, d.softwareIconD, { fontFace: HEAD_FONT_REF, fontSize: d.headPt, bold: true, color: C.text2, valign: "middle" });
		const by = CONTENT_TOP + CARD_PAD + d.softwareIconD + GAP_TIGHT;
		bullets(slide, "Software points", s.software.points, sx + CARD_PAD, by, sw - 2 * CARD_PAD, CONTENT_TOP + CONTENT_H - CARD_PAD - by, { fontSize: d.bulletPt, paraSpacePt: PARA_SPACE_LOOSE_PT });
	};

	// ---- Assemble slides ---------------------------------------------------------------------
	let currentSection = null;
	for (const s of content.slides) {
		if (s.section !== currentSection) {
			pres.addSection({ title: s.section });
			currentSection = s.section;
		}
		const masterName = s.layout === "title_dark" ? "TITLE_DARK" : "TITLE_ONLY";
		const slide = pres.addSlide({ masterName, sectionTitle: s.section });
		const build = builders[s.builder || s.id]; // changed from the registration generator, which selected by id only
		if (!build) throw new Error(`No builder "${s.builder || s.id}" for slide id "${s.id}"`);
		build(slide, s);
		slide.addNotes(s.notes);
	}

	// ---- Write, then put the theme colors into the file ---------------------------------------
	await pres.writeFile({ fileName: outputPptx });
	await applyTheme(outputPptx, THEME);
	console.log("Wrote " + outputPptx);
}

/** Command line: no arguments builds both decks; <content.json> <output.pptx> builds one. */
async function main() {
	const args = process.argv.slice(2);
	if (args.length === 0) {
		for (const [contentJson, outputPptx] of DECKS) await buildDeck(contentJson, outputPptx);
	} else if (args.length === 2) {
		await buildDeck(args[0], args[1]);
	} else {
		throw new Error("Usage: node make_perf_deck.js [<content.json> <output.pptx>]");
	}
}

main().catch((err) => {
	console.error(err);
	process.exit(1);
});
