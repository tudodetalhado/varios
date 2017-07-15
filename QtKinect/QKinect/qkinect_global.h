#ifndef QKINECT_GLOBAL_H
#define QKINECT_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(QKINECT_LIBRARY)
#  define QKINECTSHARED_EXPORT Q_DECL_EXPORT
#else
#  define QKINECTSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // QKINECT_GLOBAL_H
