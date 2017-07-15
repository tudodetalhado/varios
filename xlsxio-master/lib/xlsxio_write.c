#include "xlsxio_write.h"
#include "xlsxio_version.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <fcntl.h>
#include <stdarg.h>
#if defined(STATIC) || defined(BUILD_XLSXIO_STATIC) || defined(BUILD_XLSXIO_STATIC_DLL) || (defined(BUILD_XLSXIO) && !defined(BUILD_XLSXIO_DLL))
#define ZIP_STATIC
#endif
#include <zip.h>
#if defined(_WIN32) && !defined(USE_PTHREADS)
#define USE_WINTHREADS
#include <windows.h>
#else
#define USE_PTHREADS
#include <pthread.h>
#endif

#if defined(_MSC_VER)
#undef DLL_EXPORT_XLSXIO
#define DLL_EXPORT_XLSXIO
#endif

#ifndef ZIP_RDONLY
typedef struct zip zip_t;
typedef struct zip_source zip_source_t;
#endif

#ifdef _WIN32
#define pipe(fds) _pipe(fds, 4096, _O_BINARY)
#define read _read
#define write _write
#define write _write
#define close _close
#define fdopen _fdopen
#else
#define _fdopen(f) f
#endif

//#undef WITHOUT_XLSX_STYLES
#define DEFAULT_BUFFERED_ROWS 5

#define FONT_CHAR_WIDTH 7
//#define CALCULATE_COLUMN_WIDTH(characters) ((double)characters + .75)
#define CALCULATE_COLUMN_WIDTH(characters) ((double)(long)(((long)characters * FONT_CHAR_WIDTH + 5) * 256 / FONT_CHAR_WIDTH) / 256.0)
#define CALCULATE_COLUMN_HEIGHT(characters) ((double)characters * 12.75)

DLL_EXPORT_XLSXIO void xlsxiowrite_get_version (int* pmajor, int* pminor, int* pmicro)
{
  if (pmajor)
    *pmajor = XLSXIO_VERSION_MAJOR;
  if (pminor)
    *pminor = XLSXIO_VERSION_MINOR;
  if (pmicro)
    *pmicro = XLSXIO_VERSION_MICRO;
}

DLL_EXPORT_XLSXIO const char* xlsxiowrite_get_version_string ()
{
  return XLSXIO_VERSION_STRING;
}

////////////////////////////////////////////////////////////////////////

#ifdef USE_EXCEL_FOLDERS
#define XML_FOLDER_DOCPROPS             "docProps/"
#define XML_FOLDER_XL                   "xl/"
#define XML_FOLDER_WORKSHEETS           "worksheets/"
#else
#define XML_FOLDER_DOCPROPS             ""
#define XML_FOLDER_XL                   ""
#define XML_FOLDER_WORKSHEETS           ""
#endif
#define XML_FILENAME_CONTENTTYPES       "[Content_Types].xml"
#define XML_FILENAME_RELS               "_rels/.rels"
#define XML_FILENAME_DOCPROPS_CORE      "core.xml"
#define XML_FILENAME_DOCPROPS_APP       "app.xml"
#define XML_FILENAME_XL_WORKBOOK_RELS   "_rels/workbook.xml.rels"
#define XML_FILENAME_XL_WORKBOOK        "workbook.xml"
#define XML_FILENAME_XL_STYLES          "styles.xml"
#define XML_FILENAME_XL_WORKSHEET1      "sheet1.xml"
#define XML_SHEETNAME_MAXLEN            31

const char* content_types_xml =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
  "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">"
  "<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>"
  "<Default Extension=\"xml\" ContentType=\"application/xml\"/>"
  "<Override PartName=\"/" XML_FOLDER_XL XML_FILENAME_XL_WORKBOOK "\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml\"/>"
  "<Override PartName=\"/" XML_FOLDER_DOCPROPS XML_FILENAME_DOCPROPS_CORE "\" ContentType=\"application/vnd.openxmlformats-package.core-properties+xml\"/>"
  "<Override PartName=\"/" XML_FOLDER_DOCPROPS XML_FILENAME_DOCPROPS_APP "\" ContentType=\"application/vnd.openxmlformats-officedocument.extended-properties+xml\"/>"
#ifndef WITHOUT_XLSX_STYLES
  "<Override PartName=\"/" XML_FOLDER_XL XML_FILENAME_XL_STYLES "\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml\"/>"
#endif
  //"<Override PartName=\"/xl/theme/theme1.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.theme+xml\"/>"
  //"<Override PartName=\"/xl/sharedStrings.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.sharedStrings+xml\"/>"
  "<Override PartName=\"/" XML_FOLDER_XL XML_FOLDER_WORKSHEETS XML_FILENAME_XL_WORKSHEET1 "\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml\"/>"
  "</Types>";

const char* docprops_core_xml =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
  "<coreProperties xmlns=\"http://schemas.openxmlformats.org/package/2006/metadata/core-properties\">"
  //"<creator>" XLSXIOWRITE_FULLNAME "</creator>"
  "<lastModifiedBy>" XLSXIOWRITE_FULLNAME "</lastModifiedBy>"
  //"<modified>2016-04-24T17:50:35Z</modified>"
  "</coreProperties>";

const char* docprops_app_xml =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
  "<Properties xmlns=\"http://schemas.openxmlformats.org/officeDocument/2006/extended-properties\" xmlns:vt=\"http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes\">"
  "</Properties>";

const char* rels_xml =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
  "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
  "<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties\" Target=\"" XML_FOLDER_DOCPROPS XML_FILENAME_DOCPROPS_CORE "\"/>"
  "<Relationship Id=\"rId2\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties\" Target=\"" XML_FOLDER_DOCPROPS XML_FILENAME_DOCPROPS_APP "\"/>"
  "<Relationship Id=\"rId3\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument\" Target=\"" XML_FOLDER_XL XML_FILENAME_XL_WORKBOOK "\"/>"
  "</Relationships>";

#ifndef WITHOUT_XLSX_STYLES
const char* styles_xml =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
  "<styleSheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">\r\n"
  "<fonts count=\"2\">\r\n"
  "<font>\r\n"
  "<sz val=\"10\"/>\r\n"
  //"<color theme=\"1\"/>\r\n"
  "<name val=\"Consolas\"/>\r\n"
  "<family val=\"2\"/>\r\n"
  //"<scheme val=\"minor\"/>\r\n"
  "</font>\r\n"
  "<font>\r\n"
  "<b/><u/>"
  "<sz val=\"10\"/>\r\n"
  //"<color theme=\"1\"/>\r\n"
  "<name val=\"Consolas\"/>\r\n"
  "<family val=\"2\"/>\r\n"
  //"<scheme val=\"minor\"/>\r\n"
  "</font>\r\n"
  "</fonts>\r\n"
  "<fills count=\"1\">\r\n"
  "<fill/>\r\n"
  //"<fill><patternFill patternType=\"none\"/></fill>\r\n"
  "</fills>\r\n"
  "<borders count=\"2\">\r\n"
  "<border>\r\n"
  //"<left/>\r\n"
  //"<right/>\r\n"
  //"<top/>\r\n"
  //"<bottom/>\r\n"
  //"<diagonal/>\r\n"
  "</border>\r\n"
  "<border><bottom style=\"thin\"><color indexed=\"64\"/></bottom></border>\r\n"
  "</borders>\r\n"
  //"<cellStyleXfs count=\"1\">\r\n"
  //"<xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\"/>\r\n"
  //"</cellStyleXfs>\r\n"
  "<cellXfs count=\"6\">\r\n"
  "<xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\"/>\r\n"
#define STYLE_HEADER 1
  "<xf numFmtId=\"0\" fontId=\"1\" fillId=\"0\" borderId=\"1\" xfId=\"0\" applyFont=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\"/></xf>\r\n"
#define STYLE_GENERAL 2
  "<xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\" applyAlignment=\"1\"><alignment vertical=\"top\"/></xf>\r\n"
#define STYLE_TEXT 3
  "<xf numFmtId=\"49\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\" applyNumberFormat=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\" wrapText=\"1\"/></xf>\r\n"
#define STYLE_INTEGER 4
  "<xf numFmtId=\"1\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\" applyNumberFormat=\"1\" applyAlignment=\"1\"><alignment vertical=\"top\"/></xf>\r\n"
#define STYLE_DATETIME 5
  "<xf numFmtId=\"22\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\" applyNumberFormat=\"1\" applyAlignment=\"1\"><alignment horizontal=\"center\" vertical=\"top\"/></xf>\r\n"
  "</cellXfs>\r\n"
  //"<cellStyles count=\"2\">\r\n"
  //"<cellStyle name=\"Normal\" xfId=\"0\" builtinId=\"0\"/>\r\n"
  //"</cellStyles>\r\n"
  "<dxfs count=\"0\"/>\r\n"
  //"<tableStyles count=\"0\" defaultTableStyle=\"TableStyleMedium9\" defaultPivotStyle=\"PivotStyleLight16\"/>\r\n"
  "</styleSheet>\r\n";
#endif

/*
const char* theme_xml =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
  "<a:theme xmlns:a=\"http://schemas.openxmlformats.org/drawingml/2006/main\" name=\"Office Theme\">\r\n"
  "<a:themeElements><a:clrScheme name=\"Office\"><a:dk1><a:sysClr val=\"windowText\" lastClr=\"000000\"/></a:dk1><a:lt1><a:sysClr val=\"window\" lastClr=\"FFFFFF\"/></a:lt1><a:dk2><a:srgbClr val=\"1F497D\"/></a:dk2><a:lt2><a:srgbClr val=\"EEECE1\"/></a:lt2><a:accent1><a:srgbClr val=\"4F81BD\"/></a:accent1><a:accent2><a:srgbClr val=\"C0504D\"/></a:accent2><a:accent3><a:srgbClr val=\"9BBB59\"/></a:accent3><a:accent4><a:srgbClr val=\"8064A2\"/></a:accent4><a:accent5><a:srgbClr val=\"4BACC6\"/></a:accent5><a:accent6><a:srgbClr val=\"F79646\"/></a:accent6><a:hlink><a:srgbClr val=\"0000FF\"/></a:hlink><a:folHlink><a:srgbClr val=\"800080\"/></a:folHlink></a:clrScheme><a:fontScheme name=\"Office\"><a:majorFont><a:latin typeface=\"Cambria\"/><a:ea typeface=\"\"/><a:cs typeface=\"\"/><a:font script=\"Jpan\" typeface=\"ＭＳ Ｐゴシック\"/><a:font script=\"Hang\" typeface=\"맑은 고딕\"/><a:font script=\"Hans\" typeface=\"宋体\"/><a:font script=\"Hant\" typeface=\"新細明體\"/><a:font script=\"Arab\" typeface=\"Times New Roman\"/><a:font script=\"Hebr\" typeface=\"Times New Roman\"/><a:font script=\"Thai\" typeface=\"Tahoma\"/><a:font script=\"Ethi\" typeface=\"Nyala\"/><a:font script=\"Beng\" typeface=\"Vrinda\"/><a:font script=\"Gujr\" typeface=\"Shruti\"/><a:font script=\"Khmr\" typeface=\"MoolBoran\"/><a:font script=\"Knda\" typeface=\"Tunga\"/><a:font script=\"Guru\" typeface=\"Raavi\"/><a:font script=\"Cans\" typeface=\"Euphemia\"/><a:font script=\"Cher\" typeface=\"Plantagenet Cherokee\"/><a:font script=\"Yiii\" typeface=\"Microsoft Yi Baiti\"/><a:font script=\"Tibt\" typeface=\"Microsoft Himalaya\"/><a:font script=\"Thaa\" typeface=\"MV Boli\"/><a:font script=\"Deva\" typeface=\"Mangal\"/><a:font script=\"Telu\" typeface=\"Gautami\"/><a:font script=\"Taml\" typeface=\"Latha\"/><a:font script=\"Syrc\" typeface=\"Estrangelo Edessa\"/><a:font script=\"Orya\" typeface=\"Kalinga\"/><a:font script=\"Mlym\" typeface=\"Kartika\"/><a:font script=\"Laoo\" typeface=\"DokChampa\"/><a:font script=\"Sinh\" typeface=\"Iskoola Pota\"/><a:font script=\"Mong\" typeface=\"Mongolian Baiti\"/><a:font script=\"Viet\" typeface=\"Times New Roman\"/><a:font script=\"Uigh\" typeface=\"Microsoft Uighur\"/></a:majorFont><a:minorFont><a:latin typeface=\"Calibri\"/><a:ea typeface=\"\"/><a:cs typeface=\"\"/><a:font script=\"Jpan\" typeface=\"ＭＳ Ｐゴシック\"/><a:font script=\"Hang\" typeface=\"맑은 고딕\"/><a:font script=\"Hans\" typeface=\"宋体\"/><a:font script=\"Hant\" typeface=\"新細明體\"/><a:font script=\"Arab\" typeface=\"Arial\"/><a:font script=\"Hebr\" typeface=\"Arial\"/><a:font script=\"Thai\" typeface=\"Tahoma\"/><a:font script=\"Ethi\" typeface=\"Nyala\"/><a:font script=\"Beng\" typeface=\"Vrinda\"/><a:font script=\"Gujr\" typeface=\"Shruti\"/><a:font script=\"Khmr\" typeface=\"DaunPenh\"/><a:font script=\"Knda\" typeface=\"Tunga\"/><a:font script=\"Guru\" typeface=\"Raavi\"/><a:font script=\"Cans\" typeface=\"Euphemia\"/><a:font script=\"Cher\" typeface=\"Plantagenet Cherokee\"/><a:font script=\"Yiii\" typeface=\"Microsoft Yi Baiti\"/><a:font script=\"Tibt\" typeface=\"Microsoft Himalaya\"/><a:font script=\"Thaa\" typeface=\"MV Boli\"/><a:font script=\"Deva\" typeface=\"Mangal\"/><a:font script=\"Telu\" typeface=\"Gautami\"/><a:font script=\"Taml\" typeface=\"Latha\"/><a:font script=\"Syrc\" typeface=\"Estrangelo Edessa\"/><a:font script=\"Orya\" typeface=\"Kalinga\"/><a:font script=\"Mlym\" typeface=\"Kartika\"/><a:font script=\"Laoo\" typeface=\"DokChampa\"/><a:font script=\"Sinh\" typeface=\"Iskoola Pota\"/><a:font script=\"Mong\" typeface=\"Mongolian Baiti\"/><a:font script=\"Viet\" typeface=\"Arial\"/><a:font script=\"Uigh\" typeface=\"Microsoft Uighur\"/></a:minorFont></a:fontScheme><a:fmtScheme name=\"Office\"><a:fillStyleLst><a:solidFill><a:schemeClr val=\"phClr\"/></a:solidFill><a:gradFill rotWithShape=\"1\"><a:gsLst><a:gs pos=\"0\"><a:schemeClr val=\"phClr\"><a:tint val=\"50000\"/><a:satMod val=\"300000\"/></a:schemeClr></a:gs><a:gs pos=\"35000\"><a:schemeClr val=\"phClr\"><a:tint val=\"37000\"/><a:satMod val=\"300000\"/></a:schemeClr></a:gs><a:gs pos=\"100000\"><a:schemeClr val=\"phClr\"><a:tint val=\"15000\"/><a:satMod val=\"350000\"/></a:schemeClr></a:gs></a:gsLst><a:lin ang=\"16200000\" scaled=\"1\"/></a:gradFill><a:gradFill rotWithShape=\"1\"><a:gsLst><a:gs pos=\"0\"><a:schemeClr val=\"phClr\"><a:shade val=\"51000\"/><a:satMod val=\"130000\"/></a:schemeClr></a:gs><a:gs pos=\"80000\"><a:schemeClr val=\"phClr\"><a:shade val=\"93000\"/><a:satMod val=\"130000\"/></a:schemeClr></a:gs><a:gs pos=\"100000\"><a:schemeClr val=\"phClr\"><a:shade val=\"94000\"/><a:satMod val=\"135000\"/></a:schemeClr></a:gs></a:gsLst><a:lin ang=\"16200000\" scaled=\"0\"/></a:gradFill></a:fillStyleLst><a:lnStyleLst><a:ln w=\"9525\" cap=\"flat\" cmpd=\"sng\" algn=\"ctr\"><a:solidFill><a:schemeClr val=\"phClr\"><a:shade val=\"95000\"/><a:satMod val=\"105000\"/></a:schemeClr></a:solidFill><a:prstDash val=\"solid\"/></a:ln><a:ln w=\"25400\" cap=\"flat\" cmpd=\"sng\" algn=\"ctr\"><a:solidFill><a:schemeClr val=\"phClr\"/></a:solidFill><a:prstDash val=\"solid\"/></a:ln><a:ln w=\"38100\" cap=\"flat\" cmpd=\"sng\" algn=\"ctr\"><a:solidFill><a:schemeClr val=\"phClr\"/></a:solidFill><a:prstDash val=\"solid\"/></a:ln></a:lnStyleLst><a:effectStyleLst><a:effectStyle><a:effectLst><a:outerShdw blurRad=\"40000\" dist=\"20000\" dir=\"5400000\" rotWithShape=\"0\"><a:srgbClr val=\"000000\"><a:alpha val=\"38000\"/></a:srgbClr></a:outerShdw></a:effectLst></a:effectStyle><a:effectStyle><a:effectLst><a:outerShdw blurRad=\"40000\" dist=\"23000\" dir=\"5400000\" rotWithShape=\"0\"><a:srgbClr val=\"000000\"><a:alpha val=\"35000\"/></a:srgbClr></a:outerShdw></a:effectLst></a:effectStyle><a:effectStyle><a:effectLst><a:outerShdw blurRad=\"40000\" dist=\"23000\" dir=\"5400000\" rotWithShape=\"0\"><a:srgbClr val=\"000000\"><a:alpha val=\"35000\"/></a:srgbClr></a:outerShdw></a:effectLst><a:scene3d><a:camera prst=\"orthographicFront\"><a:rot lat=\"0\" lon=\"0\" rev=\"0\"/></a:camera><a:lightRig rig=\"threePt\" dir=\"t\"><a:rot lat=\"0\" lon=\"0\" rev=\"1200000\"/></a:lightRig></a:scene3d><a:sp3d><a:bevelT w=\"63500\" h=\"25400\"/></a:sp3d></a:effectStyle></a:effectStyleLst><a:bgFillStyleLst><a:solidFill><a:schemeClr val=\"phClr\"/></a:solidFill><a:gradFill rotWithShape=\"1\"><a:gsLst><a:gs pos=\"0\"><a:schemeClr val=\"phClr\"><a:tint val=\"40000\"/><a:satMod val=\"350000\"/></a:schemeClr></a:gs><a:gs pos=\"40000\"><a:schemeClr val=\"phClr\"><a:tint val=\"45000\"/><a:shade val=\"99000\"/><a:satMod val=\"350000\"/></a:schemeClr></a:gs><a:gs pos=\"100000\"><a:schemeClr val=\"phClr\"><a:shade val=\"20000\"/><a:satMod val=\"255000\"/></a:schemeClr></a:gs></a:gsLst><a:path path=\"circle\"><a:fillToRect l=\"50000\" t=\"-80000\" r=\"50000\" b=\"180000\"/></a:path></a:gradFill><a:gradFill rotWithShape=\"1\"><a:gsLst><a:gs pos=\"0\"><a:schemeClr val=\"phClr\"><a:tint val=\"80000\"/><a:satMod val=\"300000\"/></a:schemeClr></a:gs><a:gs pos=\"100000\"><a:schemeClr val=\"phClr\"><a:shade val=\"30000\"/><a:satMod val=\"200000\"/></a:schemeClr></a:gs></a:gsLst><a:path path=\"circle\"><a:fillToRect l=\"50000\" t=\"50000\" r=\"50000\" b=\"50000\"/></a:path></a:gradFill></a:bgFillStyleLst></a:fmtScheme></a:themeElements><a:objectDefaults/><a:extraClrSchemeLst/>\r\n"
  "</a:theme>\r\n";

const char* sharedstrings_xml =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
  "<sst xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\" count=\"35\" uniqueCount=\"34\">\r\n"
  "<si><t>Name</t></si><si><t>Expires</t></si><si><t>LastLogon</t></si><si><t>LastSetPassword</t></si><si><t>LastBadPassword</t></si><si><t>LockOut</t></si><si><t>Active</t></si><si><t>Created</t></si><si><t>LastChanged</t></si><si><t>Login</t></si><si><t>Logons</t></si><si><t>ADName</t></si><si><t>FirstName</t></si><si><t>LastName</t></si><si><t>PrimaryEmail</t></si><si><t>HomeDirectory</t></si><si><t>Notes</t></si><si><t>PasswordExpires</t></si><si><t/></si><si><t>Never</t></si><si><t>Firefighter (Realdolmen)</t></si><si><t>2016-02-28 00:00:00</t></si><si><t>2016-02-17 12:00:00</t></si><si><t>2015-12-24 09:41:29</t></si><si><t>2016-02-01 11:50:11</t></si><si><t>Enabled+Expired</t></si><si><t>2013-02-27 10:30:55</t></si><si><t>2016-02-26 08:51:27</t></si><si><t>zxBeweRD008_f</t></si><si><t>CN=Firefighter (Realdolmen),OU=Admin Accounts,OU=Domain Security,DC=ISIS,DC=LOCAL</t></si><si><t>Realdolmen</t></si><si><t>Firefighter</t></si><si><t>Realdolmen.Firefighter@alpro.com</t></si><si><t>consultant RealDolmen - managed services - no often used</t></si>\r\n"
  "</sst>\r\n";
*/

const char* workbook_rels_xml =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
  "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
  //"<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/theme\" Target=\"theme/theme1.xml\"/>"
#ifndef WITHOUT_XLSX_STYLES
  "<Relationship Id=\"rId2\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles\" Target=\"" XML_FILENAME_XL_STYLES "\"/>"
#endif
  "<Relationship Id=\"rId3\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet\" Target=\"" XML_FOLDER_WORKSHEETS XML_FILENAME_XL_WORKSHEET1 "\"/>"
  //"<Relationship Id=\"rId4\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/sharedStrings\" Target=\"sharedStrings.xml\"/>"
  "</Relationships>";

const char* workbook_xml =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
  "<workbook xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\" xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\">"
  //"<workbookPr/>"
  "<bookViews>"
  "<workbookView/>"
  "</bookViews>"
  "<sheets>"
  "<sheet name=\"%s\" sheetId=\"1\" r:id=\"rId3\"/>"
  "</sheets>"
  "</workbook>";

const char* worksheet_xml_begin =
  "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\r\n"
  "<worksheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\" xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\">";

const char* worksheet_xml_freeze_top_row =
  "<sheetViews>"
  "<sheetView tabSelected=\"1\" workbookViewId=\"0\">"
  "<pane ySplit=\"1\" topLeftCell=\"A2\" activePane=\"bottomLeft\" state=\"frozen\"/>"
  "<selection pane=\"bottomLeft\"/>"
  "</sheetView>"
  "</sheetViews>";

const char* worksheet_xml_start_data =
  "<sheetData>";

const char* worksheet_xml_end =
  "</sheetData>"
  //"<pageMargins left=\"0.75\" right=\"0.75\" top=\"1\" bottom=\"1\" header=\"0.5\" footer=\"0.5\"/>"
  "</worksheet>";

////////////////////////////////////////////////////////////////////////

int zip_add_content_buffer (zip_t* zip, const char* filename, const char* buf, size_t buflen, int mustfree)
{
  zip_source_t* zipsrc;
  if ((zipsrc = zip_source_buffer(zip, buf, buflen, mustfree)) == NULL) {
    fprintf(stderr, "Error creating file \"%s\" inside zip file\n", filename);/////
    return 1;
  }
  if (zip_file_add(zip, filename, zipsrc, ZIP_FL_OVERWRITE | ZIP_FL_ENC_UTF_8) < 0) {
    fprintf(stderr, "Error in zip_file_add\n");/////
    zip_source_free(zipsrc);
    return 2;
  }
  return 0;
}

int zip_add_static_content_string (zip_t* zip, const char* filename, const char* data)
{
  return zip_add_content_buffer(zip, filename, data, strlen(data), 0);
}

int zip_add_dynamic_content_string (zip_t* zip, const char* filename, const char* data, ...)
{
  int result;
  char* buf;
  int buflen;
  va_list args;
  va_start(args, data);
  buflen = vsnprintf(NULL, 0, data, args);
  if (buflen < 0 || (buf = (char*)malloc(buflen + 1)) == NULL) {
    result = -1;
  } else {
    va_end(args);
    va_start(args, data);
    vsnprintf(buf, buflen + 1, data, args);
    result = zip_add_content_buffer(zip, filename, buf, buflen, 1);
  }
  va_end(args);
  return result;
}

////////////////////////////////////////////////////////////////////////

//replace part of a string
char* str_replace (char** s, size_t pos, size_t len, char* replacement)
{
  if (!s || !*s)
    return NULL;
  size_t totallen = strlen(*s);
  size_t replacementlen = strlen(replacement);
  if (pos > totallen)
    pos = totallen;
  if (pos + len > totallen)
    len = totallen - pos;
  if (replacementlen > len)
    if ((*s = (char*)realloc(*s, totallen - len + replacementlen + 1)) == NULL)
      return NULL;
  memmove(*s + pos + replacementlen, *s + pos + len, totallen - pos - len + 1);
  memcpy(*s + pos, replacement, replacementlen);
  return *s;
}

//fix string for use as XML data
char* fix_xml_special_chars (char** s)
{
	int pos = 0;
	while (*s && (*s)[pos]) {
		switch ((*s)[pos]) {
			case '&' :
        str_replace(s, pos, 1, "&amp;");
				pos += 5;
				break;
			case '\"' :
				str_replace(s, pos, 1, "&quot;");
				pos += 6;
				break;
			case '<' :
				str_replace(s, pos, 1, "&lt;");
				pos += 4;
				break;
			case '>' :
				str_replace(s, pos, 1, "&gt;");
				pos += 4;
				break;
			case '\r' :
				str_replace(s, pos, 1, "");
				break;
			default:
				pos++;
				break;
		}
	}
	return *s;
}

////////////////////////////////////////////////////////////////////////

struct column_info_struct {
  int width;
  int maxwidth;
  struct column_info_struct* next;
};

struct xlsxio_write_struct {
  char* filename;
  char* sheetname;
  zip_t* zip;
#ifdef USE_WINTHREADS
  HANDLE thread;
#else
  pthread_t thread;
#endif
  FILE* pipe_read;
  FILE* pipe_write;
  struct column_info_struct* columninfo;
  struct column_info_struct** pcurrentcolumn;
  char* buf;
  size_t buflen;
  size_t rowstobuffer;
  size_t rowheight;
  int freezetop;
  int sheetopen;
  int rowopen;
};

//thread function used for creating .xlsx file from pipe
#ifdef USE_WINTHREADS
DWORD WINAPI thread_proc (LPVOID arg)
#else
void* thread_proc (void* arg)
#endif
{
  xlsxiowriter handle = (xlsxiowriter)arg;
  //initialize zip file object
  if ((handle->zip = zip_open(handle->filename, ZIP_CREATE, NULL)) == NULL) {
    free(handle);
    free(handle->filename);
#ifdef USE_WINTHREADS
    return 0;
#else
    return NULL;
#endif
  }
  //generate required files
  zip_add_static_content_string(handle->zip, XML_FILENAME_CONTENTTYPES, content_types_xml);
  zip_add_static_content_string(handle->zip, XML_FOLDER_DOCPROPS XML_FILENAME_DOCPROPS_CORE, docprops_core_xml);
  zip_add_static_content_string(handle->zip, XML_FOLDER_DOCPROPS XML_FILENAME_DOCPROPS_APP, docprops_app_xml);
  zip_add_static_content_string(handle->zip, XML_FILENAME_RELS, rels_xml);
#ifndef WITHOUT_XLSX_STYLES
  zip_add_static_content_string(handle->zip, XML_FOLDER_XL XML_FILENAME_XL_STYLES, styles_xml);
#endif
  //zip_add_static_content_string(handle->zip, "xl/theme/theme1.xml", theme_xml);
  zip_add_static_content_string(handle->zip, XML_FOLDER_XL XML_FILENAME_XL_WORKBOOK_RELS, workbook_rels_xml);
  { //TO DO: this crashes on Linux
    char* sheetname = NULL;
    if (handle->sheetname) {
      sheetname = strdup(handle->sheetname);
      if (sheetname) {
        if (strlen(sheetname) > XML_SHEETNAME_MAXLEN)
          sheetname[XML_SHEETNAME_MAXLEN] = 0;
        fix_xml_special_chars(&sheetname);
      }
    }
    zip_add_dynamic_content_string(handle->zip, XML_FOLDER_XL XML_FILENAME_XL_WORKBOOK, workbook_xml, (sheetname ? sheetname : "Sheet1"));
    free(sheetname);
  }
  //zip_add_static_content_string(handle->zip, "xl/sharedStrings.xml", sharedstrings_xml);

  //add sheet content with pipe data as source
  zip_source_t* zipsrc = zip_source_filep(handle->zip, handle->pipe_read, 0, -1);
  if (zip_file_add(handle->zip, XML_FOLDER_XL XML_FOLDER_WORKSHEETS XML_FILENAME_XL_WORKSHEET1, zipsrc, ZIP_FL_OVERWRITE | ZIP_FL_ENC_UTF_8) < 0) {
    zip_source_free(zipsrc);
    fprintf(stdout, "Error adding file");
  }
#ifdef ZIP_RDONLY
  zip_file_set_mtime(handle->zip, zip_get_num_entries(handle->zip, 0) - 1, time(NULL), 0);
#endif

  //close zip file (processes all data, will block until pipe is closed)
  if (zip_close(handle->zip) != 0) {
    int ze, se;
#ifdef ZIP_RDONLY
    zip_error_t* error = zip_get_error(handle->zip);
    ze = zip_error_code_zip(error);
    se = zip_error_code_system(error);
#else
    zip_error_get(handle->zip, &ze, &se);
#endif
    fprintf(stderr, "zip_close failed (%i,%i)\n", ze, se);/////
    fprintf(stderr, "can't close zip archive : %s\n", zip_strerror(handle->zip));
  }
  handle->zip = NULL;
  handle->pipe_read = NULL;
#ifdef USE_WINTHREADS
  return 0;
#else
  return NULL;
#endif
}

////////////////////////////////////////////////////////////////////////

DLL_EXPORT_XLSXIO xlsxiowriter xlsxiowrite_open (const char* filename, const char* sheetname)
{
  xlsxiowriter handle;
  if (!filename)
    return NULL;
  if ((handle = (xlsxiowriter)malloc(sizeof(struct xlsxio_write_struct))) != NULL) {
    int pipefd[2];
    //initialize
    handle->filename = strdup(filename);
    handle->sheetname = (sheetname ? strdup(sheetname) : NULL);
    handle->zip = NULL;
    //handle->pipe_read = NULL;
    //handle->pipe_write = NULL;
    handle->columninfo = NULL;
    handle->pcurrentcolumn = &handle->columninfo;
    handle->buf = NULL;
    handle->buflen = 0;
    handle->rowstobuffer = DEFAULT_BUFFERED_ROWS;
    handle->rowheight = 0;
    handle->freezetop = 0;
    handle->sheetopen = 0;
    handle->rowopen = 0;
    //create pipe
    if (pipe(pipefd) != 0) {
      fprintf(stderr, "Error creating pipe\n");/////
      free(handle);
      return NULL;
    }
    handle->pipe_read = fdopen(pipefd[0], "rb");
    handle->pipe_write = fdopen(pipefd[1], "wb");
    //remove filename first if it already exists
    unlink(filename);
    //create and start thread that will receive data via pipe
#ifdef USE_WINTHREADS
    if ((handle->thread = CreateThread(NULL, 0, thread_proc, handle, 0, NULL)) == NULL) {
#else
    if (pthread_create(&handle->thread, NULL, thread_proc, handle) != 0) {
#endif
      fprintf(stderr, "Error creating thread\n");/////
    }
    //write initial worksheet data
    fprintf(handle->pipe_write, "%s", worksheet_xml_begin);
  }
  return handle;
}

void flush_buffer (xlsxiowriter handle);

DLL_EXPORT_XLSXIO int xlsxiowrite_close (xlsxiowriter handle)
{
  struct column_info_struct* colinfo;
  struct column_info_struct* colinfonext;
  if (!handle)
    return -1;
  //finalize data
  if (handle->pipe_write) {
    //check if buffer should be flushed
    if (!handle->sheetopen)
      flush_buffer(handle);
    //close row if needed
    if (handle->rowopen)
      fprintf(handle->pipe_write, "</row>");
    //write worksheet data
    fprintf(handle->pipe_write, "%s", worksheet_xml_end);
    //close pipe
    fclose(handle->pipe_write);
  }
  //wait for thread to finish
#ifdef USE_WINTHREADS
  WaitForSingleObject(handle->thread, INFINITE);
#else
  pthread_join(handle->thread, NULL);
#endif
  //clean up
  colinfo = handle->columninfo;
  while (colinfo) {
    colinfonext = colinfo->next;
    free(colinfo);
    colinfo = colinfonext;
  }
  free(handle->filename);
  free(handle->sheetname);
  if (handle->zip)
    zip_close(handle->zip);
  if (handle->pipe_read)
    fclose(handle->pipe_read);
  free(handle);
  return 0;
}

#ifndef WITHOUT_XLSX_STYLES
#define STYLE_ATTR_HELPER(x) #x
#define STYLE_ATTR(style) " s=\"" STYLE_ATTR_HELPER(style) "\""
#else
#define STYLE_ATTR(style) ""
#endif

//add data to a null-terminated buffer and update the length counter
int vappend_data (char** pdata, size_t* pdatalen, const char* format, va_list args)
{
  int len;
  //va_start(args, format);
  va_list args2;
  va_copy(args2, args);
  if ((len = vsnprintf(NULL, 0, format, args)) < 0)
    return -1;
  va_end(args);
  if ((*pdata = (char*)realloc(*pdata, *pdatalen + len + 1)) == NULL)
    return -1;
  vsnprintf(*pdata + *pdatalen, len + 1, format, args2);
  va_end(args2);
  *pdatalen += len;
  return len;
}

//add formatted data to a null-terminated buffer and update the length counter
int append_data (char** pdata, size_t* pdatalen, const char* format, ...)
{
  int result;
  va_list args;
  va_start(args, format);
  result = vappend_data(pdata, pdatalen, format, args);
  va_end(args);
  return result;
}

//output start of row
void write_row_start (xlsxiowriter handle, const char* rowattr)
{
  if (handle->sheetopen) {
    if (!handle->rowheight)
      fprintf(handle->pipe_write, "<row%s>", (rowattr ? rowattr : ""));
    else
      fprintf(handle->pipe_write, "<row ht=\"%.6G\" customHeight=\"1\"%s>", CALCULATE_COLUMN_HEIGHT(handle->rowheight), (rowattr ? rowattr : ""));
  } else {
    if (!handle->rowheight)
      append_data(&handle->buf, &handle->buflen, "<row%s>", (rowattr ? rowattr : ""));
    else
      append_data(&handle->buf, &handle->buflen, "<row ht=\"%.6G\" customHeight=\"1\"%s>",  CALCULATE_COLUMN_HEIGHT(handle->rowheight), (rowattr ? rowattr : ""));
  }
  handle->rowopen = 1;
}

//output cell data
void write_cell_data (xlsxiowriter handle, const char* rowattr, const char* prefix, const char* suffix, const char* format, ...)
{
  va_list args;
  if (!handle)
    return;
  //start new row if needed
  if (!handle->rowopen)
    write_row_start(handle, rowattr);
  //get formatted data
  int datalen;
  char* data;
  va_start(args, format);
  if (format && (datalen = vsnprintf(NULL, 0, format, args)) >= 0 && (data = (char*)malloc(datalen + 1)) != NULL) {
    va_end(args);
    va_start(args, format);
    vsnprintf(data, datalen + 1, format, args);
    //prepare data for XML output
    fix_xml_special_chars(&data);
  } else {
    data = NULL;
    datalen = 0;
  }
  va_end(args);
  //add cell data
  if (handle->sheetopen) {
    //write cell data
    if (prefix)
      fprintf(handle->pipe_write, "%s", prefix);
    if (data)
      fprintf(handle->pipe_write, "%s", data);
    if (suffix)
      fprintf(handle->pipe_write, "%s", suffix);
  } else {
    //add cell data to buffer
    if (prefix)
      append_data(&handle->buf, &handle->buflen, "%s", prefix);
    if (data)
      append_data(&handle->buf, &handle->buflen, "%s", data);
    if (suffix)
      append_data(&handle->buf, &handle->buflen, "%s", suffix);
    //collect cell information
    if (!handle->sheetopen) {
      if (!*handle->pcurrentcolumn) {
        //create new column information structure
        struct column_info_struct* colinfo;
        if ((colinfo = (struct column_info_struct*)malloc(sizeof(struct column_info_struct))) != NULL) {
          colinfo->width = 0;
          colinfo->maxwidth = 0;
          colinfo->next = NULL;
          *handle->pcurrentcolumn = colinfo;
        }
      }
      //keep track of biggest column width
      if (data) {
        //only count first line in multiline data
        char* p = strchr(data, '\n');
        if (p)
          datalen = p - data;
        //remember this length if it is the longest one so far
        if (datalen > 0 && datalen > (*handle->pcurrentcolumn)->maxwidth)
          (*handle->pcurrentcolumn)->maxwidth = datalen;
      }
      //prepare for the next column
      handle->pcurrentcolumn = &(*handle->pcurrentcolumn)->next;
    }
  }
  free(data);
}

//output buffered data and stop buffering
void flush_buffer (xlsxiowriter handle)
{
  //write section to freeze top row
  if (handle->freezetop > 0)
    fprintf(handle->pipe_write, "%s", worksheet_xml_freeze_top_row);
  //default to row height of 1 line
  //fprintf(handle->pipe_write, "<sheetFormatPr defaultRowHeight=\"%.6G\" customHeight=\"1\"/>", (double)12.75);
  //write column information
  if (handle->columninfo) {
    int col = 0;
    int len;
    struct column_info_struct* colinfo = handle->columninfo;
    fprintf(handle->pipe_write, "<cols>");
    while (colinfo) {
      ++col;
      //determine column width
      len = colinfo->width;
      if (len == 0) {
        //use detected maximum length if column width specified was zero
        if (colinfo->maxwidth > 0)
          len = colinfo->maxwidth;
      } else if (len < 0) {
        //use detected maximum length if column width specified was negative and the detected maximum length is larger than the absolute value of the specified width
        len = -len;
        if (colinfo->maxwidth > len)
          len = colinfo->maxwidth;
      }
      if (len)
        fprintf(handle->pipe_write, "<col min=\"%i\" max=\"%i\" width=\"%.6G\" customWidth=\"1\"/>", col, col, CALCULATE_COLUMN_WIDTH(len));
      else
        fprintf(handle->pipe_write, "<col min=\"%i\" max=\"%i\"/>", col, col);
      colinfo = colinfo->next;
    }
    fprintf(handle->pipe_write, "</cols>");
  }
  //write initial data
  fprintf(handle->pipe_write, "%s", worksheet_xml_start_data);
  //write buffer and clear it
  if (handle->buf) {
    if (handle->buflen > 0)
      fwrite(handle->buf, 1, handle->buflen, handle->pipe_write);
    free(handle->buf);
    handle->buf = NULL;
  }
  handle->buflen = 0;
  handle->sheetopen = 1;
}

DLL_EXPORT_XLSXIO void xlsxiowrite_set_detection_rows (xlsxiowriter handle, size_t rows)
{
  //abort if currently not buffering
  if (!handle->rowstobuffer || handle->sheetopen)
    return;
  //set number of rows to buffer
  handle->rowstobuffer = rows;
  //flush when zero was specified
  if (!rows)
    flush_buffer(handle);
}

DLL_EXPORT_XLSXIO void xlsxiowrite_set_row_height (xlsxiowriter handle, size_t height)
{
  handle->rowheight = height;
}

DLL_EXPORT_XLSXIO void xlsxiowrite_add_column (xlsxiowriter handle, const char* value, int width)
{
  struct column_info_struct** pcolinfo = handle->pcurrentcolumn;
  if (value)
    write_cell_data(handle, STYLE_ATTR(STYLE_HEADER), "<c t=\"inlineStr\"" STYLE_ATTR(STYLE_HEADER) "><is><t>", "</t></is></c>", "%s", value);
  else
    write_cell_data(handle, STYLE_ATTR(STYLE_HEADER), "<c" STYLE_ATTR(STYLE_HEADER) "/>", NULL, NULL);
  if (*pcolinfo)
    (*pcolinfo)->width = width;
  if (handle->freezetop == 0)
    handle->freezetop = 1;
}

DLL_EXPORT_XLSXIO void xlsxiowrite_add_cell_string (xlsxiowriter handle, const char* value)
{
  if (value)
    write_cell_data(handle, NULL, "<c t=\"inlineStr\"" STYLE_ATTR(STYLE_TEXT) "><is><t>", "</t></is></c>", "%s", value);
  else
    write_cell_data(handle, NULL, "<c" STYLE_ATTR(STYLE_TEXT) "/>", NULL, NULL);
}

DLL_EXPORT_XLSXIO void xlsxiowrite_add_cell_int (xlsxiowriter handle, int64_t value)
{
  write_cell_data(handle, NULL, "<c" STYLE_ATTR(STYLE_INTEGER) "><v>", "</v></c>", "%" PRIi64, value);
}

DLL_EXPORT_XLSXIO void xlsxiowrite_add_cell_float (xlsxiowriter handle, double value)
{
  write_cell_data(handle, NULL, "<c" STYLE_ATTR(STYLE_GENERAL) "><v>", "</v></c>", "%.32G", value);
}

DLL_EXPORT_XLSXIO void xlsxiowrite_add_cell_datetime (xlsxiowriter handle, time_t value)
{
  double timestamp = ((double)(value) + .499) / 86400 + 25569; //conversion from Unix to Excel timestamp
  write_cell_data(handle, NULL, "<c" STYLE_ATTR(STYLE_DATETIME) "><v>", "</v></c>", "%.16G", timestamp);
}
/*
Windows (And Mac Office 2011+):

    Unix Timestamp = (Excel Timestamp - 25569) * 86400
    Excel Timestamp = (Unix Timestamp / 86400) + 25569

MAC OS X (pre Office 2011):

    Unix Timestamp = (Excel Timestamp - 24107) * 86400
    Excel Timestamp = (Unix Timestamp / 86400) + 24107
*/

DLL_EXPORT_XLSXIO void xlsxiowrite_next_row (xlsxiowriter handle)
{
  if (!handle)
    return;
  //check if buffer should be flushed
  if (!handle->sheetopen) {
    if (handle->rowstobuffer > 0) {
      if (--handle->rowstobuffer == 0) {
        flush_buffer(handle);
      } else {
      }
    }
  }
  //start new row if needed
  if (!handle->rowopen)
    write_row_start(handle, NULL);
  //end row
  if (handle->rowstobuffer == 0)
    fprintf(handle->pipe_write, "</row>");
  else
    append_data(&handle->buf, &handle->buflen, "</row>");
  handle->rowopen = 0;
  handle->pcurrentcolumn = &handle->columninfo;
}

