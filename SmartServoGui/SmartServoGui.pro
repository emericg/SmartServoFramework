#-------------------------------------------------------------------------------
# SmartServoGui build system
#-------------------------------------------------------------------------------

TARGET      = SmartServoGui
DESTDIR     = bin/

TEMPLATE    = app
CONFIG     += c++11
QT         += core svg gui widgets serialport

RCC_DIR     = build/
UI_DIR      = build/
MOC_DIR     = build/
OBJECTS_DIR = build/

## QtSerialPort
#QT_VERSION = $$[QT_VERSION]
#QT_VERSION = $$split(QT_VERSION, ".")
#QT_VER_MAJ = $$member(QT_VERSION, 0)
#QT_VER_MIN = $$member(QT_VERSION, 1)
#contains(QT_VER_MAJ, 5) | greaterThan(QT_VER_MIN, 6) {
#   QT += serialport
#   DEFINES += FEATURE_QTSERIAL
#   message(Using QtSerialPort instead of OS specific implementation.)
#}

# OS specifics build settings
unix {
    QMAKE_CXXFLAGS += -fPIE

    # Enables AddressSanitizer
    #QMAKE_CXXFLAGS += -fsanitize=address,undefined
    #QMAKE_LFLAGS += -fsanitize=address,undefined

    *-g++* {
        message("Using GCC compiler")
        QMAKE_CXXFLAGS += -pthread
        QMAKE_CXXFLAGS += -Wno-unused-parameter -Wno-unused-variable
    }
    *clang* {
        message("Using LLVM compiler")
        QMAKE_CXXFLAGS += -Wno-unused-parameter -Wno-unused-variable
    }

    linux {
        message("Building on Linux/BSD plateform")
    }
    macx {
        message("Building on macOS plateform")
        LIBS += -framework IOKit -framework CoreFoundation

        # Force compiler to use available macOS SDK version (with automatic detection)
        XCODE_SDK_VERSION = $$system("xcodebuild -sdk macosx -version | grep SDKVersion | cut -d' ' -f2-")
        QMAKE_MAC_SDK = "macosx$${XCODE_SDK_VERSION}"
        #QMAKE_MACOSX_DEPLOYMENT_TARGET = $${XCODE_SDK_VERSION}

        # Force RPATH to look into the 'Frameworks' dir (doesn't really seems to work...)
        #QMAKE_RPATHDIR += @executable_path/../Frameworks
    }
}
win32 {
    message("Building on Windows plateform")

    *-g++* {
        message("Using MinGW compiler")
        QMAKE_CXXFLAGS += -pthread -Wno-unused-parameter -Wno-unused-variable
    }
    *-msvc* {
        message("Using MSVC compiler")
    }
}

# SmartServoFramework
contains(CONFIG, SSF_SYSTEM) {
    # Use SmartServoFramework library from the system
    unix {
        INCLUDEPATH += `pkg-config --cflags smartservoframework`
        LIBS += `pkg-config --libs smartservoframework`
    }
    win32 {
        SSF_DIR = ..
        INCLUDEPATH += $${SSF_DIR}
        LIBS += $${SSF_DIR}/build/smartservoframework.lib
    }
} else {
    # Use SmartServoFramework sources directly
    SSF_DIR = ..
    INCLUDEPATH += $${SSF_DIR}
    SOURCES += $${SSF_DIR}/SmartServoFramework/*.cpp
    HEADERS += $${SSF_DIR}/SmartServoFramework/*.h
}

# SmartServoGui sources
SOURCES    += src/main.cpp \
              src/mainwindow.cpp \
              src/tabSerial.cpp \
              src/advancescanner.cpp \
              src/widgetSerialScan.cpp \
              src/widgetSerialError.cpp \
              src/widgetRegisterTable.cpp \
              src/qabout.cpp \
              src/settings.cpp \
              src/protocolcombobox.cpp \
              src/baudratecombobox.cpp

HEADERS    += src/mainwindow.h \
              src/tabSerial.h \
              src/advancescanner.h \
              src/widgetSerialScan.h \
              src/widgetSerialError.h \
              src/widgetRegisterTable.h \
              src/qabout.h \
              src/settings.h \
              src/protocolcombobox.h \
              src/baudratecombobox.h

FORMS      += ui/mainwindow.ui \
              ui/advancescanner.ui \
              ui/qabout.ui \
              ui/settings.ui \
              ui/tabSerial.ui \
              ui/widgetSerialScan.ui \
              ui/widgetSerialError.ui \
              ui/widgetRegisterTable.ui

RESOURCES  += resources/resources.qrc

ICON        = resources/app/smartservogui.icns
RC_ICONS    = resources/app/smartservogui.ico

# Use "lupdate SmartServoGui.pro" to update translation files
# Then "lrelease SmartServoGui.pro" to build translated files
TRANSLATIONS = resources/lang/es.ts resources/lang/fr.ts resources/lang/it.ts

# Deployment -------------------------------------------------------------------

win32 {
    # Application packaging
    #system(windeployqt $${OUT_PWD}/$${DESTDIR})

    # Automatic application packaging
    deploy.commands = $$quote(windeployqt $${OUT_PWD}/$${DESTDIR}/)
    install.depends = deploy
    QMAKE_EXTRA_TARGETS += install deploy

    # Installation
    # TODO?

    # Clean bin/ directory
    # TODO
}

macx {
    # Bundle packaging
    #system(macdeployqt $${OUT_PWD}/$${DESTDIR}/$${TARGET}.app)

    # Automatic bundle packaging
    deploy.commands = macdeployqt $${OUT_PWD}/$${DESTDIR}/$${TARGET}.app
    install.depends = deploy
    QMAKE_EXTRA_TARGETS += install deploy

    # Installation
    target.files += $${OUT_PWD}/${DESTDIR}/${TARGET}.app
    target.path = $$(HOME)/Applications
    INSTALLS += target

    # Clean bin/ directory
    QMAKE_DISTCLEAN += -r $${OUT_PWD}/${DESTDIR}/${TARGET}.app
}

linux {
    TARGET = $$lower($${TARGET})

    # Application packaging
    #system(linuxdeployqt $${OUT_PWD}/$${DESTDIR}/) # needs linuxdeployqt installed

    # Installation
    isEmpty(PREFIX) { PREFIX = /usr/local }
    target_app.files   += $${OUT_PWD}/$${DESTDIR}/$$lower($${TARGET})
    target_app.path     = $${PREFIX}/bin/
    target_icon.files  += $${OUT_PWD}/resources/app/$$lower($${TARGET}).svg
    target_icon.path    = $${PREFIX}/share/pixmaps/
    target_appentry.files  += $${OUT_PWD}/resources/app/$$lower($${TARGET}).desktop
    target_appentry.path    = $${PREFIX}/share/applications
    target_appdata.files   += $${OUT_PWD}/resources/app/$$lower($${TARGET}).appdata.xml
    target_appdata.path     = $${PREFIX}/share/appdata
    INSTALLS += target_app target_icon target_appentry target_appdata

    # Clean bin/ directory
    #QMAKE_CLEAN += $${OUT_PWD}/$${DESTDIR}/$$lower($${TARGET})
}
