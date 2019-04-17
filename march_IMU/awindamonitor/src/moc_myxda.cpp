/****************************************************************************
** Meta object code from reading C++ file 'myxda.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../include/awindamonitor/myxda.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'myxda.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MyXda[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
       7,    6,    6,    6, 0x05,
      42,    6,    6,    6, 0x05,
      72,    6,    6,    6, 0x05,
      96,    6,    6,    6, 0x05,
     127,    6,    6,    6, 0x05,

 // slots: signature, parameters, type, tag, flags
     154,    6,    6,    6, 0x0a,
     162,    6,    6,    6, 0x0a,
     183,    6,    6,    6, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_MyXda[] = {
    "MyXda\0\0wirelessMasterDetected(XsPortInfo)\0"
    "dockedMtwDetected(XsPortInfo)\0"
    "mtwUndocked(XsPortInfo)\0"
    "openPortSuccessful(XsPortInfo)\0"
    "openPortFailed(XsPortInfo)\0reset()\0"
    "openPort(XsPortInfo)\0scanPorts()\0"
};

inline void MyXda::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MyXda *_t = static_cast<MyXda *>(_o);
        switch (_id) {
        case 0: _t->wirelessMasterDetected((*reinterpret_cast< XsPortInfo(*)>(_a[1]))); break;
        case 1: _t->dockedMtwDetected((*reinterpret_cast< XsPortInfo(*)>(_a[1]))); break;
        case 2: _t->mtwUndocked((*reinterpret_cast< XsPortInfo(*)>(_a[1]))); break;
        case 3: _t->openPortSuccessful((*reinterpret_cast< XsPortInfo(*)>(_a[1]))); break;
        case 4: _t->openPortFailed((*reinterpret_cast< XsPortInfo(*)>(_a[1]))); break;
        case 5: _t->reset(); break;
        case 6: _t->openPort((*reinterpret_cast< XsPortInfo(*)>(_a[1]))); break;
        case 7: _t->scanPorts(); break;
        default: ;
        }
    }
}

//const QMetaObjectExtraData MyXda::staticMetaObjectExtraData = {
//    0,  qt_static_metacall 
//};

//const QMetaObject MyXda::staticMetaObject = {
//    { &QObject::staticMetaObject, qt_meta_stringdata_MyXda,
//      qt_meta_data_MyXda, &staticMetaObjectExtraData }
////};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MyXda::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

inline const QMetaObject *MyXda::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

inline void *MyXda::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MyXda))
        return static_cast<void*>(const_cast< MyXda*>(this));
    return QObject::qt_metacast(_clname);
}

inline int MyXda::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
inline void MyXda::wirelessMasterDetected(XsPortInfo _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
inline void MyXda::dockedMtwDetected(XsPortInfo _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
inline void MyXda::mtwUndocked(XsPortInfo _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
inline void MyXda::openPortSuccessful(XsPortInfo _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
inline void MyXda::openPortFailed(XsPortInfo _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
static const uint qt_meta_data_MyWirelessMasterCallback[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      13,       // signalCount

 // signals: signature, parameters, type, tag, flags
      28,   26,   25,   25, 0x05,
      66,   25,   25,   25, 0x05,
      97,   25,   25,   25, 0x05,
     128,   25,   25,   25, 0x05,
     165,   25,   25,   25, 0x05,
     194,   25,   25,   25, 0x05,
     222,   25,   25,   25, 0x05,
     246,   25,   25,   25, 0x05,
     271,   25,   25,   25, 0x05,
     295,   25,   25,   25, 0x05,
     315,   25,   25,   25, 0x05,
     338,   25,   25,   25, 0x05,
     363,  359,   25,   25, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_MyWirelessMasterCallback[] = {
    "MyWirelessMasterCallback\0\0,\0"
    "deviceError(XsDeviceId,XsResultValue)\0"
    "measurementStarted(XsDeviceId)\0"
    "measurementStopped(XsDeviceId)\0"
    "waitingForRecordingStart(XsDeviceId)\0"
    "recordingStarted(XsDeviceId)\0"
    "mtwDisconnected(XsDeviceId)\0"
    "mtwRejected(XsDeviceId)\0"
    "mtwPluggedIn(XsDeviceId)\0"
    "mtwWireless(XsDeviceId)\0mtwFile(XsDeviceId)\0"
    "mtwUnknown(XsDeviceId)\0mtwError(XsDeviceId)\0"
    ",,,\0progressUpdate(XsDeviceId,int,int,QString)\0"
};

inline void MyWirelessMasterCallback::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MyWirelessMasterCallback *_t = static_cast<MyWirelessMasterCallback *>(_o);
        switch (_id) {
        case 0: _t->deviceError((*reinterpret_cast< XsDeviceId(*)>(_a[1])),(*reinterpret_cast< XsResultValue(*)>(_a[2]))); break;
        case 1: _t->measurementStarted((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 2: _t->measurementStopped((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 3: _t->waitingForRecordingStart((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 4: _t->recordingStarted((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 5: _t->mtwDisconnected((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 6: _t->mtwRejected((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 7: _t->mtwPluggedIn((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 8: _t->mtwWireless((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 9: _t->mtwFile((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 10: _t->mtwUnknown((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 11: _t->mtwError((*reinterpret_cast< XsDeviceId(*)>(_a[1]))); break;
        case 12: _t->progressUpdate((*reinterpret_cast< XsDeviceId(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        default: ;
        }
    }
}

//const QMetaObjectExtraData MyWirelessMasterCallback::staticMetaObjectExtraData = {
//    0,  qt_static_metacall 
//};

//const QMetaObject MyWirelessMasterCallback::staticMetaObject = {
//    { &QObject::staticMetaObject, qt_meta_stringdata_MyWirelessMasterCallback,
//      qt_meta_data_MyWirelessMasterCallback, &staticMetaObjectExtraData }
//};

#ifdef Q_NO_DATA_RELOCATION
inline const QMetaObject &MyWirelessMasterCallback::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

inline const QMetaObject *MyWirelessMasterCallback::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

inline void *MyWirelessMasterCallback::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MyWirelessMasterCallback))
        return static_cast<void*>(const_cast< MyWirelessMasterCallback*>(this));
    if (!strcmp(_clname, "XsCallback"))
        return static_cast< XsCallback*>(const_cast< MyWirelessMasterCallback*>(this));
    return QObject::qt_metacast(_clname);
}

inline int MyWirelessMasterCallback::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
inline void MyWirelessMasterCallback::deviceError(XsDeviceId _t1, XsResultValue _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
inline void MyWirelessMasterCallback::measurementStarted(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
inline void MyWirelessMasterCallback::measurementStopped(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
inline void MyWirelessMasterCallback::waitingForRecordingStart(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
inline void MyWirelessMasterCallback::recordingStarted(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
inline void MyWirelessMasterCallback::mtwDisconnected(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
inline void MyWirelessMasterCallback::mtwRejected(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
inline void MyWirelessMasterCallback::mtwPluggedIn(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
inline void MyWirelessMasterCallback::mtwWireless(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
inline void MyWirelessMasterCallback::mtwFile(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
inline void MyWirelessMasterCallback::mtwUnknown(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
inline void MyWirelessMasterCallback::mtwError(XsDeviceId _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
inline void MyWirelessMasterCallback::progressUpdate(XsDeviceId _t1, int _t2, int _t3, QString _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}
static const uint qt_meta_data_MyMtwCallback[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      17,   15,   14,   14, 0x05,
      56,   15,   14,   14, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_MyMtwCallback[] = {
    "MyMtwCallback\0\0,\0"
    "dataAvailable(XsDeviceId,XsDataPacket)\0"
    "batteryLevelChanged(XsDeviceId,int)\0"
};

inline void MyMtwCallback::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MyMtwCallback *_t = static_cast<MyMtwCallback *>(_o);
        switch (_id) {
        case 0: _t->dataAvailable((*reinterpret_cast< XsDeviceId(*)>(_a[1])),(*reinterpret_cast< XsDataPacket(*)>(_a[2]))); break;
        case 1: _t->batteryLevelChanged((*reinterpret_cast< XsDeviceId(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        default: ;
        }
    }
}

//const QMetaObjectExtraData MyMtwCallback::staticMetaObjectExtraData = {
//    0,  qt_static_metacall 
//};

//const QMetaObject MyMtwCallback::staticMetaObject = {
//    { &QObject::staticMetaObject, qt_meta_stringdata_MyMtwCallback,
//      qt_meta_data_MyMtwCallback, &staticMetaObjectExtraData }
//};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MyMtwCallback::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

inline const QMetaObject *MyMtwCallback::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

inline void *MyMtwCallback::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MyMtwCallback))
        return static_cast<void*>(const_cast< MyMtwCallback*>(this));
    if (!strcmp(_clname, "XsCallback"))
        return static_cast< XsCallback*>(const_cast< MyMtwCallback*>(this));
    return QObject::qt_metacast(_clname);
}

inline int MyMtwCallback::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
inline void MyMtwCallback::dataAvailable(XsDeviceId _t1, XsDataPacket _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
inline void MyMtwCallback::batteryLevelChanged(XsDeviceId _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
