#ifndef DOUBLESLIDERDESIGNER_H
#define DOUBLESLIDERDESIGNER_H

#include <QtDesigner/QDesignerCustomWidgetInterface>
#include <QtDesigner/QDesignerExportWidget>

class QDESIGNER_WIDGET_EXPORT QDoubleSliderDesigner : public QObject, public QDesignerCustomWidgetInterface
{
  Q_OBJECT
  Q_INTERFACES(QDesignerCustomWidgetInterface)

 public:

  QDoubleSliderDesigner(QObject *parent = 0);

  bool isContainer() const;
  bool isInitialized() const;
  QIcon icon() const;
  QString domXml() const;
  QString group() const;
  QString includeFile() const;
  QString name() const;
  QString toolTip() const;
  QString whatsThis() const;
  QWidget *createWidget(QWidget *parent);
  void initialize(QDesignerFormEditorInterface *core);

 private:
  bool initialized_;
};

#endif // !DOUBLESLIDERDESIGNER_H
