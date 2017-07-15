#include "qDoubleRangeSlider.h"
#include <cmath>

#include <QtGui/QHBoxLayout>
#include <cassert>
#include <stdexcept>


QDoubleRangeSlider::QDoubleRangeSlider(QWidget* parent)
  : QWidget(parent),
    slider_(new QRangeSlider(this))
{
  setup();
}

QDoubleRangeSlider::QDoubleRangeSlider(Qt::Orientation orientation,
                                       QWidget* parent)
  : QWidget(parent),
    slider_(new QRangeSlider(orientation, this))
{
  setup();
}

void QDoubleRangeSlider::setup()
{
  //GUI elements
  QLayout* layout = new QHBoxLayout(this);
  setLayout(layout);
  layout->addWidget(slider_);
  layout->setContentsMargins(0,0,0,0);

  //Simple values, with no invariant dependencies
  slider_->setUnitConverter(this);
  isLogarithmic_ = false;

  //Values with interdependencies
  cutoffRange_ = numericalLimits();
  range_ = cutoffRange_;
  resetInternalCutoffRange();
  resetInternalRange();

  setTickInterval(10.0);

  setFixedHeight(16);

  //Connections
  bool ok = true;
  ok &= connect(slider_,SIGNAL(rangeChanged(QPair<int,int>)),
                this,SLOT(rangeChanged(QPair<int,int>)));
  assert(ok);
}


void QDoubleRangeSlider::resetInternalCutoffRange()
{
  slider_->setCutoffRange(QPair<int, int>(RANGE_SLIDER_MIN, RANGE_SLIDER_MAX));
}

void QDoubleRangeSlider::resetInternalRange()
{
  slider_->setRange(QPair<int, int>(RANGE_SLIDER_MIN, RANGE_SLIDER_MAX));
}


bool
QDoubleRangeSlider::cmp(int a, int b, uint offset)
{
  return ((a+int(offset))>=b)
    && ((a-int(offset))<=b);
}

bool
QDoubleRangeSlider::cmp(const QPair<int, int>& a,
                   const QPair<int, int>& b,
                   uint offset) {
  return cmp(a.first, b.first, offset)&&
    cmp(a.second, b.second, offset);
}

void
QDoubleRangeSlider::setRange(QPair<double, double> range)
{
  clamp(range, cutoffRange());
  if (range_ == range)
    return;

  range_ = range;

  //Need to allow for perfect values
  QPair<int, int> nRange = convertToBase(range);
  QPair<int, int> oRange = slider_->range();

  expectValue_ = nRange;

  if (!cmp(oRange,nRange,1)) {
    slider_->setRange(nRange);
    emit rangeChanged(range);
  }
}

void
QDoubleRangeSlider::setCutoffRange(QPair<double, double> cutoffRange)
{
  clamp(cutoffRange, numericalLimits());
  if (cutoffRange_ == cutoffRange)
    return;

  QPair<double, double> oRange = range();
  cutoffRange_ = cutoffRange;

  if (cutoffRange.first == cutoffRange.second) {
    slider_->setCutoffRange(QPair<int,int>(0, 0));
  }
  else {
    resetInternalCutoffRange();
    resetInternalRange();
  }

  setRange(oRange);
  emit cutoffRangeChanged(cutoffRange_);
}


QDoubleRangeSlider::range_t
QDoubleRangeSlider::range() const
{
  return range_;
}


QDoubleRangeSlider::range_t
QDoubleRangeSlider::cutoffRange() const
{
  return cutoffRange_;
}


void
QDoubleRangeSlider::setTickInterval(double tickInterval)
{
  tickInterval_ = tickInterval;
  tickInterval = std::log(tickInterval);
  double max = cutoffRange_.second;
  double min = cutoffRange_.first;
  if (isLogarithmic()) {
    max = std::log(max);
    min = std::log(min);
  }
  double div =  max - min;

  tickInterval = (tickInterval)/div * RANGE_SLIDER_DIV;

  slider_->setTickInterval(tickInterval);
}

double QDoubleRangeSlider::convertFromBaseToDouble(int value) const
{
  QPair<int, int> sliderMaxRange = slider_->cutoffRange();

  int first  = sliderMaxRange.first;
  int second = sliderMaxRange.second;

  int div = second - first;

  if (div == 0) {
    assert(cutoffRange_.first == cutoffRange_.second);
    return cutoffRange_.first;
  }

  double retVal = double(value - first) / div;

  double offset = cutoffRange_.first;

  double max = cutoffRange_.second;

  if(isLogarithmic_) {
    offset = std::log(offset);
    max = std::log(max);
  }
  double range = (max - offset);

  retVal = retVal * range + offset;

  return (isLogarithmic_?std::exp(retVal):retVal);
}

QVariant QDoubleRangeSlider::convertFromBase(int value) const
{
  return QString().setNum(convertFromBaseToDouble(value), 'g', 5);
}


QPair<double, double>
QDoubleRangeSlider::convertFromBase(QPair<int, int> value) const
{

  return QPair<double, double>(convertFromBaseToDouble(value.first),
                               convertFromBaseToDouble(value.second));
}


void
QDoubleRangeSlider::rangeChanged(QPair<int, int> value)
{
  if (cutoffRange_.second == cutoffRange_.first)
    return;

  QPair<double, double> range = convertFromBase(value);

  if (!cmp(value,expectValue_,1)) {
    range_ = range;
    emit rangeChanged(range);
  }
}

int QDoubleRangeSlider::convertToBase(double value) const
{
  double retVal = (value-cutoffRange_.first)/
    (cutoffRange_.second-cutoffRange_.first);


  QPair<int, int> sliderMaxRange = slider_->cutoffRange();
  if (isLogarithmic()) {
    double offset = cutoffRange_.first;
    double max = cutoffRange_.second;
    value -= offset;
    if(isLogarithmic_) {
      /*
      offset = log(offset);
      max = log(max);
      */
    }
    double range = (max - offset);

    retVal = (retVal - offset)/range;


    return int(retVal);
  }
  return int(retVal * (sliderMaxRange.second - sliderMaxRange.first) +
             sliderMaxRange.first);
}


int QDoubleRangeSlider::convertToBase(QVariant value) const
{
  bool isOk = false;

  double val = value.toDouble(&isOk);

  if (!isOk)
    throw std::domain_error("Invalid value specified, expected double");

  return convertToBase(val);
}

QPair<int, int>
QDoubleRangeSlider::convertToBase(QPair<double, double> value) const
{
  return QPair<int, int>(convertToBase(value.first),
                        convertToBase(value.second));
}

bool QDoubleRangeSlider::clamp(QPair<double, double>& value,
                          const QPair<double, double>& limits)
{
  bool changed = false;
  changed |= clamp(value.first, limits);
  changed |= clamp(value.second, limits);
  return changed;
}

bool QDoubleRangeSlider::clamp(double& value,
                               const QPair<double, double>& limits)
{
  assert(limits.first <= limits.second);
  if (value < limits.first) {
    value = limits.first;
    return true;
  }
  if (value > limits.second) {
    value = limits.second;
    return true;
  }
  return false;
}


bool QDoubleRangeSlider::isLogarithmic() const
{
  return isLogarithmic_;
}


void QDoubleRangeSlider::setLogarithmic(bool logaritmic)
{
  isLogarithmic_ = logaritmic;
  setTickInterval(tickInterval_);
}
