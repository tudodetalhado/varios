

//1. Creating data sheets
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <Xlsx/Workbook.h>

using namespace std;
using namespace SimpleXlsx;

int main()
{
   srand(time(NULL));
   const int colNum = 20;
   const int rowNum = 10;

   CWorkbook book;

   {    // Creating a simple data sheet
        CWorksheet &sheet = book.AddSheet(_T("New sheet simple"));

        vector<CellDataDbl> data;   // (data:style_index)
        CellDataDbl def;
        def.style_id = 0;       // 0,1 - default styles
        for (int i = 0; i < colNum; i++) {
            def.value = (double)(rand() % 100) / 101.0;
            data.push_back(def);
        }

        for (int i = 0; i < rowNum; i++)
            sheet.AddRow(data);
   }

   {    // Creating data sheet with a frozen pane
        CWorksheet &sheet = book.AddSheet(_T("New sheet with frozen pane"), 0, 1);

        vector<CellDataStr> dataStr;// (data:style_index)
        CellDataStr col;

        col.value = _T("Frozen pane header");
        col.style_id = 0;       // 0,1 - default styles
        dataStr.push_back(col);
        sheet.AddRow(dataStr);
        sheet.MergeCells(CellCoord(1, 0), CellCoord(1, colNum-1)); // merge first row

        vector<CellDataInt> data;   // (data:style_index)
        CellDataInt def;
        def.style_id = 0;       // 0,1 - default styles
        for (int i = 0; i < colNum; i++) {
            def.value = (double)(rand() % 100);
            data.push_back(def);
        }

        for (int i = 0; i < rowNum; i++)
            sheet.AddRow(data);
   }

   {    // Creating data sheet with col and row specified sizes
        std::vector<ColumnWidth> colWidths;
        for (int i = 0; i < 50; i++) {
            ColumnWidth colWidth;
            colWidth.colFrom = colWidth.colTo = i;
            colWidth.width = i + 10;
            colWidths.push_back(colWidth);
        }

        CWorksheet &sheet = book.AddSheet(_T("New sheet with column and row specified sizes"), colWidths);

        Style style;
        style.wrapText = true;
        int style_index = book.m_styleList.Add(style);

        vector<CellDataStr> data;   // (data:style_index)
        CellDataStr def;
        def.style_id = style_index; // 0,1 - default styles
        data.push_back(def);
            TCHAR szText[30] = { 0 };
        for (int i = 0; i < colNum; i++) {
            _stprintf(szText, _T("some\nmultirow\ntext %d_%d"), i+10, i);
            def.value = szText;
            data.push_back(def);
        }

        for (int i = 0; i < rowNum; i++)
            sheet.AddRow(data, 0, 45);
   }

    {   // Creating data sheet with styled cells
        CWorksheet &sheet = book.AddSheet(_T("New sheet with styled cells"));

        Style style;
        style.fill.patternType = PATTERN_NONE;
        style.font.size = 14;
        style.font.theme = true;
        style.font.attributes = FONT_BOLD;
        style.horizAlign = ALIGN_H_RIGHT;
        style.vertAlign = ALIGN_V_CENTER;
        int style_index_1 = book.m_styleList.Add(style);

        style.fill.patternType = PATTERN_NONE;
        style.font.size = 14;
        style.font.theme = true;
        style.font.attributes = FONT_ITALIC;
        style.horizAlign = ALIGN_H_LEFT;
        style.vertAlign = ALIGN_V_CENTER;
        int style_index_2 = book.m_styleList.Add(style);

        vector<CellDataFlt> data;
        CellDataFlt def;
        for (int i = 0; i < colNum; i++) {
            def.value = (double)(rand() % 100) / 101.0;
            if (i % 2 == 0) def.style_id = style_index_1;
            else        def.style_id = style_index_2;

            data.push_back(def);
        }

        for (int i = 0; i < rowNum; i++)
            sheet.AddRow(data, 5);
   }

   bool bRes = book.Save(_T("MyBook.xlsx"));
   if (bRes)   cout << "The book has been saved successfully";
   else        cout << "The book saving has been failed";

   return 0;
}

// 2. Creating chart sheets
#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <Xlsx/Workbook.h>

using namespace std;
using namespace SimpleXlsx;

int main()
{
    // main object - workbook
    CWorkbook book;

    // data sheet to store data to be referenced
    CWorksheet &sheet = book.AddSheet(_T("Data"));

    vector<CellDataDbl> data;
    CellDataDbl cellDbl;
    cellDbl.style_id = 0;

    // fill data sheet with randomly generated numbers
    srand(time(0));
    for (int i = 0; i < 15; i++) {
        cellDbl.value = (rand() % 100) / 50.0;
        data.push_back(cellDbl);
    }

    for (int i = 0; i < 20; i++)
        sheet.AddRow(data); // data can be added by row or by cell

    /*  addition cell-by-cell may be useful if there are different data types in one row
        to perform it the code will look like:
        sheet.BeginRow();

        sheet.AddCell(cellStr);
        sheet.AddCell();
        sheet.AddCells(cellRangeData);
        ....

        sheet.EndRow();
    */

    // create series object, that contains most chart settings
    CChartsheet::Series ser;

    // adding chart to the workbook the reference to a newly created object is returned
    CChartsheet &line_chart = book.AddChart(_T("Line Chart"), CHART_LINEAR);

    // leave category sequence (X axis) not specified (optional) - MS Excel will generate the default sequence automatically
    ser.catSheet =  NULL;

    // specify range for values` sequence (Y axis)
    ser.valAxisFrom = CellCoord(0,0);
    ser.valAxisTo = CellCoord(0, 10);
    ser.valSheet =  &sheet; // don`t forget to set the pointer to the data sheet

    ser.title = _T("Line series test");
    ser.isSmoothed = true;  // determines whether series will be a smoothed or straight-lined curve
    ser.isMarked = false;   // if true add diamond marks in each node of the sequence set

    // add series into the chart (you can add as many series as you wish into the same chart)
    line_chart.AddSeries(ser);

    // adding chart to the workbook the reference to a newly created object is returned
    CChartsheet &bar_chart = book.AddChart(_T("Bar Chart"), CHART_BAR);

    // leave category sequence (X axis) not specified (optional) - MS Excel will generate the default sequence automatically
    ser.catSheet =  NULL;

    // specify range for values` sequence (Y axis)
    ser.valAxisFrom = CellCoord(0,0);
    ser.valAxisTo = CellCoord(0, 10);
    ser.valSheet =  &sheet; // don`t forget to set the pointer to the data sheet
    ser.title = _T("Bar series test");

    // optionally it is possible to set some additional parameters for bar chart
    bar_chart.SetBarDirection(CChartsheet::BAR_DIR_HORIZONTAL);
    bar_chart.SetBarGrouping(CChartsheet::BAR_GROUP_CLUSTERED);

    bar_chart.SetTableDataState(CChartsheet::TBL_DATA);

    // add series into the chart (you can add as many series as you wish into the same chart)
    bar_chart.AddSeries(ser);

    // adding chart to the workbook the reference to a newly created object is returned
    CChartsheet &scatter_chart = book.AddChart(_T("Scatter Chart"), CHART_SCATTER);
    // for scatter charts it is obligatory to specify both category (X axis) and values (Y axis) sequences
    ser.catAxisFrom = CellCoord(2,1);
    ser.catAxisTo = CellCoord(2, 11);
    ser.catSheet =  &sheet;

    ser.valAxisFrom = CellCoord(0,0);
    ser.valAxisTo = CellCoord(0, 10);
    ser.valSheet =  &sheet;

    // optional parameters
    ser.title = _T("Scatter series test");
    ser.isSmoothed = true;  // determines whether series will be a smoothed or straight-lined curve
    ser.isMarked = true;    // if true add diamond marks in each node of the sequence set
    scatter_chart.SetScatterStyle(CChartsheet::SCATTER_FILL);

    // add series into the chart (you can add as many series as you wish into the same chart)
    scatter_chart.AddSeries(ser);

    // optional parameters to set
    scatter_chart.SetLegendPos(CChartsheet::POS_RIGHT);
    scatter_chart.SetXAxisGrid(CChartsheet::GRID_MAJOR_N_MINOR);
    scatter_chart.SetYAxisGrid(CChartsheet::GRID_MAJOR_N_MINOR);
    //scatter_chart.SetTableDataState(CChartsheet::TBL_DATA); - it is not possible to add table into scatter-contained chart
    // due to specificy of the data
    scatter_chart.SetDiagrammName(_T("Scatter curves` chart"));

    // at the end save created workbook wherever you need
    bool bRes = book.Save(_T("MyBook.xlsx"));
    if (bRes) cout << "Book saved successfully";
    else cout << "Error at book saving. Operation failed";

    return 0;
}

//3. Creating numeric styles
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <Xlsx/Workbook.h>

using namespace std;
using namespace SimpleXlsx;

void addSample(CWorksheet &sheet, int style_index);
void addSampleDateTime(CWorksheet &sheet, int style_index);

int main()
{
srand(time(NULL));

CWorkbook book;
CWorksheet &sheet = book.AddSheet(_T("Styled sheet"));

vector<CellDataStr> data(1);

Style style;

data[0] = _T("Simple numeric format");
sheet.AddRow(data);
style.numFormat.numberStyle = NUMSTYLE_NUMERIC;
style.numFormat.numberOfDigitsAfterPoint = 5;
style.numFormat.showThousandsSeparator = false;
style.numFormat.positiveColor = NUMSTYLE_COLOR_GREEN;
style.numFormat.negativeColor = NUMSTYLE_COLOR_RED;

addSample(sheet, book.m_styleList.Add(style));

data[0] = _T("Financial format");
sheet.AddRow(data);
style.numFormat.numberStyle = NUMSTYLE_FINANCIAL;
style.numFormat.numberOfDigitsAfterPoint = 2;
style.numFormat.showThousandsSeparator = true;
style.numFormat.positiveColor = NUMSTYLE_COLOR_BLUE;
style.numFormat.negativeColor = NUMSTYLE_COLOR_DEFAULT;

addSample(sheet, book.m_styleList.Add(style));

data[0] = _T("Exponential format");
sheet.AddRow(data);
style.numFormat.numberStyle = NUMSTYLE_EXPONENTIAL;
style.numFormat.numberOfDigitsAfterPoint = 3;
style.numFormat.showThousandsSeparator = true;
style.numFormat.positiveColor = NUMSTYLE_COLOR_DEFAULT;
style.numFormat.negativeColor = NUMSTYLE_COLOR_DEFAULT;

addSample(sheet, book.m_styleList.Add(style));

data[0] = _T("Percentage format");
sheet.AddRow(data);
style.numFormat.numberStyle = NUMSTYLE_PERCENTAGE;
style.numFormat.numberOfDigitsAfterPoint = 0;
style.numFormat.showThousandsSeparator = false;
style.numFormat.positiveColor = NUMSTYLE_COLOR_DEFAULT;
style.numFormat.negativeColor = NUMSTYLE_COLOR_RED;

addSample(sheet, book.m_styleList.Add(style));

data[0] = _T("Money format");
sheet.AddRow(data);
style.numFormat.numberStyle = NUMSTYLE_MONEY;
style.numFormat.numberOfDigitsAfterPoint = 10;
style.numFormat.showThousandsSeparator = true;
style.numFormat.positiveColor = NUMSTYLE_COLOR_GREEN;
style.numFormat.negativeColor = NUMSTYLE_COLOR_DEFAULT;

addSample(sheet, book.m_styleList.Add(style));

data[0] = _T("Datetime format");
sheet.AddRow(data);
style.numFormat.numberStyle = NUMSTYLE_DATETIME;
style.numFormat.numberOfDigitsAfterPoint = 0;           // not used for this (and TIME and DATE) format(s)
style.numFormat.showThousandsSeparator = false;         // not used for this (and TIME and DATE) format(s)
style.numFormat.positiveColor = NUMSTYLE_COLOR_DEFAULT; // not used for this (and TIME and DATE) format(s)
style.numFormat.negativeColor = NUMSTYLE_COLOR_DEFAULT; // not used for this (and TIME and DATE) format(s)

addSampleDateTime(sheet, book.m_styleList.Add(style));

// If format string is set, it will be taken as format without any processing
// When creating a format, do not forget about xml specific symbols, e.g. symbol '"' => '&quot;' etc.
data[0] = _T("Custom financial format");
sheet.AddRow(data);
style.numFormat.formatString = _T("_-* #,##0.0000&quot;$&quot;_-;[Red]\\-* #,##0.0000&quot;$&quot;_-;_-* &quot;-&quot;??&quot;$&quot;_-;_-@_-");

addSample(sheet, book.m_styleList.Add(style));

bool bRes = book.Save(_T("MyBook.xlsx"));
if (bRes)   cout << "The book has been saved successfully";
else        cout << "The book saving has been failed";

return 0;
}

void addSample(CWorksheet &sheet, int style_index)
{
const int colNum = 20;
const int rowNum = 10;

vector<CellDataFlt> data;
CellDataFlt def;

def.style_id = style_index;

for (int i = 0; i < rowNum; i++) {
    for (int i = 0; i < colNum; i++) {
        def = ((double)(rand() % 100) / 101.0);
        if (rand() % 100 < 50) def.value *= -1;
        data.push_back(def);
    }

    sheet.AddRow(data);
    data.clear();
}

sheet.AddRow(data);
}

void addSampleDateTime(CWorksheet &sheet, int style_index)
{
const int colNum = 20;
const int rowNum = 10;

vector<CellDataTime> data;
CellDataTime def;

def.style_id = style_index;

for (int i = 0; i < rowNum; i++) {
    for (int i = 0; i < colNum; i++) {
        def = time(NULL);
        data.push_back(def);
    }

    sheet.AddRow(data);
    data.clear();
}

sheet.AddRow(data);
}

//

