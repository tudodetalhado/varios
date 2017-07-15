#include "qDoubleRangeSlider.h"
#include <QtCore/QSettings>
#include <QtGui/QApplication>
#include <QtGui/QMainWindow>
#include <QtGui/QMessageBox>
#include <QtCore/QDebug>

#include <iostream>

QString
tabs(int numTabs) {
  QString retVal;
  for (int i=1;i<numTabs;++i) {
    retVal+="  ";
  }
  return retVal;
}

void showElements(QSettings& settings, int level, QTextStream& out)
{
  QStringList keys=settings.childKeys();
  foreach(QString key, keys) {
    QVariant value=settings.value(key);
    QString rep =
      value.canConvert(QVariant::String) ?
      value.toString() :
      "<Cannot display>";
    out << tabs(level + 1) + key << "=" << '"' << rep << '"' << "\n";
  }
}

QSettings* loadSettings(QStringList& args) {
  QString filename;

  QSettings* settings = 0;

  for (int i = 0; i < args.length(); ++i) {
    if (args[i] == "-f") {
      filename = args.takeAt(i + 1);
      args.takeAt(i);
    }
  }

  if (!filename.isEmpty()) {
    settings = new QSettings(filename, QSettings::NativeFormat);
  }

  return settings;
}

void
show(QSettings& settings, const QString& path, int level, QTextStream& out)
{
  settings.beginGroup(path);
  out << tabs(level) + path << "\n";
  showElements(settings, level, out);
  QStringList children=settings.childGroups();
  foreach(QString child,  children) {
    show(settings, child, level + 1, out);
  }
  settings.endGroup();
}

 int main(int argc, char* argv[])
 {
     QApplication app(argc, argv);
     app.setOrganizationName("Numerical Rocks");
     app.setApplicationName("Settings editor");

     if (argc<3) {
       QMessageBox msgBox;
       QString errorText =
         QString("Usage is:\n\t%1 <Company> <Application>").arg(argv[0]);
       qDebug()<<errorText;
       msgBox.setText(errorText);
       msgBox.exec();
       exit(1);
     }

     QStringList args;
     for (int i = 0; i < argc; ++i) {
       args << argv[i];
     }

     QSettings* settings = loadSettings(args);

     if(!settings) {
       std::cerr << "Unable to load settings";
       return 1;
     }

     QString command = args[1];
     QString key;
     QString value;
;
     if (args.size() > 2) {
       key = args[2];
     }
     if (args.size() > 3) {
       value = args[3];
     }

     QTextStream out(stdout);

     if(command == "show") {
       if (key.isEmpty()) {
         show(*settings, "", 0, out);
         out.flush();
       }
       else {
         QString value = settings->value(key).toString();
         out << key << "=\"" << value << "\"\n";
         out.flush();
       }
     }
     else if(command == "set") {
       QVariant oldValue = settings->value(key);
       if(!oldValue.isNull() && !oldValue.canConvert(QVariant::String)) {
         std::cerr <<
           "You are trying to set a non-representable value with a string."
           "This is not supported";
         exit(1);
       }

       settings->setValue(key, value);
     }
     else {
       out << QString("Invalid command %1\n").arg(command);
       out.flush();
     }

     if(settings != 0)
       delete settings;
     exit(0);
 }
