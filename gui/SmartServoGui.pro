#-------------------------------------------------
#
# Project created by QtCreator 2014-03-21T10:06:09
#
#-------------------------------------------------

TARGET      = SmartServoGui
TEMPLATE    = app
CONFIG     += qt
QT         += core gui widgets

DESTDIR     = build/
RCC_DIR     = build/
UI_DIR      = build/
MOC_DIR     = build/
OBJECTS_DIR = build/

# OS specifics build settings:
unix {
    *-g++* {
        message("Using GCC")
        QMAKE_CXXFLAGS += -pthread

        if: system("gcc -dumpversion | grep 4.[0-5]") {
            error("You need at least GCC 4.6+ to use C++11 features")
        } else: system("gcc -dumpversion | grep 4.6") {
            QMAKE_CXXFLAGS += -std=c++0x -Wno-unused-parameter -Wno-unused-variable
        } else: {
            QMAKE_CXXFLAGS += -std=c++11 -Wno-unused-parameter -Wno-unused-variable
        }
    }
    *clang* {
        message("Using LLVM")
        QMAKE_CXXFLAGS += -std=c++11 -stdlib=libc++ -Wno-unused-parameter -Wno-unused-variable
        LIBS += -stdlib=libc++
    }

    unix:!macx {
        LIBS += -llockdev
    }
    unix:macx {
        LIBS += -framework IOKit -framework CoreFoundation
    }
}
win32 {
    *-g++* {
        message("Using MinGW / Windows")
        QMAKE_CXXFLAGS += -std=c++11 -pthread -Wno-unused-parameter -Wno-unused-variable
    }
    *-msvc* {
        message("Using MSVC / Windows")
    }
}

# SmartServoFramework sources
SOURCES    += ../src/*.cpp
HEADERS    += ../src/*.h

# GUI application sources
SOURCES    += src/main.cpp src/mainwindow.cpp src/advancescanner.cpp src/qabout.cpp src/settings.cpp
HEADERS    += src/mainwindow.h src/advancescanner.h src/qabout.h src/settings.h

RESOURCES  += resources/resources.qrc

FORMS      += ui/mainwindow.ui \
              ui/advancescanner.ui \
              ui/qabout.ui \
              ui/settings.ui

# Use "lupdate SmartServoGui.pro" to update translation files
# Then "lrelease SmartServoGui.pro" to build translated files
TRANSLATIONS = resources/lang/es.ts resources/lang/fr.ts resources/lang/it.ts
