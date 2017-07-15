#ifndef RANGESLIDERDESIGNER_H
#define RANGESLIDERDESIGNER_H

#include <QtDesigner/QDesignerCustomWidgetInterface>
#include <QtDesigner/QDesignerExportWidget>

class QDESIGNER_WIDGET_EXPORT QRangeSliderDesigner : public QObject, public QDesignerCustomWidgetInterface
{
  Q_OBJECT
  Q_INTERFACES(QDesignerCustomWidgetInterface)

 public:

  QRangeSliderDesigner(QObject *parent = 0);

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

#endif // !RANGESLIDERDESIGNER_H
