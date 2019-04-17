/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../include/awindamonitor/mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      23,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      27,   11,   11,   11, 0x08,
      56,   48,   11,   11, 0x08,
      77,   11,   11,   11, 0x08,
      97,   11,   11,   11, 0x08,
     115,   11,   11,   11, 0x08,
     132,   11,   11,   11, 0x08,
     162,   11,   11,   11, 0x08,
     185,   11,   11,   11, 0x08,
     226,   11,   11,   11, 0x08,
     262,   11,   11,   11, 0x08,
     292,   11,   11,   11, 0x08,
     329,   11,   11,   11, 0x08,
     362,   11,   11,   11, 0x08,
     399,   11,   11,   11, 0x08,
     438,  436,   11,   11, 0x08,
     476,   11,   11,   11, 0x08,
     519,   11,   11,   11, 0x08,
     558,  554,   11,   11, 0x08,
     607,   11,   11,   11, 0x08,
     637,   11,   11,   11, 0x08,
     671,  436,   11,   11, 0x08,
     716,  436,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0closeLogFile()\0"
    "toggleRadioEnabled()\0channel\0"
    "setRadioChannel(int)\0toggleMeasurement()\0"
    "toggleRecording()\0clearLogWindow()\0"
    "clearConnectedMtwDataLabels()\0"
    "requestBatteryLevels()\0"
    "handleWirelessMasterDetected(XsPortInfo)\0"
    "handleDockedMtwDetected(XsPortInfo)\0"
    "handleMtwUndocked(XsPortInfo)\0"
    "handleOpenPortSuccessful(XsPortInfo)\0"
    "handleOpenPortFailed(XsPortInfo)\0"
    "handleMeasurementStarted(XsDeviceId)\0"
    "handleMeasurementStopped(XsDeviceId)\0"
    ",\0handleError(XsDeviceId,XsResultValue)\0"
    "handleWaitingForRecordingStart(XsDeviceId)\0"
    "handleRecordingStarted(XsDeviceId)\0"
    ",,,\0handleProgressUpdate(XsDeviceId,int,int,QString)\0"
    "handleMtwWireless(XsDeviceId)\0"
    "handleMtwDisconnected(XsDeviceId)\0"
    "handleDataAvailable(XsDeviceId,XsDataPacket)\0"
    "handleBatteryLevelChanged(XsDeviceId,int)\0"
};

inline void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->closeLogFile(); break;
        case 1: _t->toggleRadioEnabled(); break;
        case 2: _t->setRadioChannel((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->toggleMeasurement(); break;
        case 4: _t->toggleRecording(); break;
        case 5: _t->clearLogWindow(); break;
        case 6: _t->clearConnectedMtwDataLabels(); break;
        case 7: _t->requestBatteryLevels(); break;
        case 8: _t->handleWirelessMasterDetected((*reinterpret_cast< XsPortInfo(*)>(_a[1]))); break;
        case 9: _t->handleDockedMtwDetected((*reinterpret_cast< XsPortInfo(*)>(_a[1]))); break;
        case 10: _t->handleMtwUndocked((*reinterpret_cast< XsPortInfo(*)>(_a[1]))); break;
        case 11: _t->handleOpenPortSuccessful((*reinterpret_cast< const XsPortInfo(*)>(_a[1]))); break;
        case 12: _t->handleOpenPortFailed((*reinterpret_cast< const XsPortInfo(*)>(_a[1]))); break;
        case 13: _t->handleMeasurementStarted((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 14: _t->handleMeasurementStopped((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 15: _t->handleError((*reinterpret_cast< XsDeviceId(*)>(_a[1])),(*reinterpret_cast< XsResultValue(*)>(_a[2]))); break;
        case 16: _t->handleWaitingForRecordingStart((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 17: _t->handleRecordingStarted((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 18: _t->handleProgressUpdate((*reinterpret_cast< XsDeviceId(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 19: _t->handleMtwWireless((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 20: _t->handleMtwDisconnected((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 21: _t->handleDataAvailable((*reinterpret_cast< XsDeviceId(*)>(_a[1])),(*reinterpret_cast< XsDataPacket(*)>(_a[2]))); break;
        case 22: _t->handleBatteryLevelChanged((*reinterpret_cast< XsDeviceId(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        default: ;
        }
    }
}

//const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
//    0,  qt_static_metacall 
//};

//	const QMetaObject MainWindow::staticMetaObject = {
//	    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
//	      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
//	};

	#ifdef Q_NO_DATA_RELOCATION
	const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
	#endif //Q_NO_DATA_RELOCATION

	inline const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

inline void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

inline int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 23)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 23;
    }
    return _id;
}

// SIGNAL 0
inline void MainWindow::closeLogFile()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
