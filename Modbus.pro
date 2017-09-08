TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += include

HEADERS += \
    include/commons.h \
    include/heap_memory.h \
    include/modbus_common.h \
    include/modbus_rtu_client.h

SOURCES += \
    src/commons.c \
    src/heap_memory.c \
    src/main.c \
    src/modbus_rtu_client.c
