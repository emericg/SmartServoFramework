#-------------------------------------------------
# SmartServoGui
# Project created by QtCreator 2014-03-21T10:06:09
#-------------------------------------------------

TARGET      = SmartServoGui
TEMPLATE    = app
CONFIG     += qt
QT         += core svg gui widgets

DESTDIR     = build/
RCC_DIR     = build/
UI_DIR      = build/
MOC_DIR     = build/
OBJECTS_DIR = build/

# OS specifics build settings
unix {
    *-g++* {
        message("Using GCC compiler")
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
        message("Using LLVM compiler")
        QMAKE_CXXFLAGS += -std=c++11 -stdlib=libc++ -Wno-unused-parameter -Wno-unused-variable
        LIBS += -stdlib=libc++
    }

    unix:!macx {
        message("Building on Linux/BSD plateform")
        LIBS += -llockdev
    }
    unix:macx {
        message("Building on MAC OS X plateform")
        QMAKE_LFLAGS += -F /System/Library/Frameworks/
        LIBS += -framework IOKit -framework CoreFoundation

        # Force RPATH to look into the 'Frameworks' dir? Doesn't really seems to work...
        #QMAKE_RPATHDIR += @executable_path/../Frameworks
        #QMAKE_LFLAGS   += -Wl,-rpath,@executable_path/../Frameworks

        # Force Qt to use a particular SDK version
        #QMAKE_MAC_SDK = macosx10.11
        #QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.9
    }
}
win32 {
    message("Building on Windows plateform")

    *-g++* {
        message("Using MinGW compiler")
        QMAKE_CXXFLAGS += -std=c++11 -pthread -Wno-unused-parameter -Wno-unused-variable
    }
    *-msvc* {
        message("Using MSVC compiler")
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



# Mac OS X deploy rules:
# (Or you can just use macdeployqt...)
unix:macx {
    FW_DIR = build/$${TARGET}.app/Contents/Frameworks
    QT_DIR = /usr/local/lib/

    # Copy libraries into the package
    QMAKE_POST_LINK += (mkdir -p $${FW_DIR})
    #QMAKE_POST_LINK += && (cp ../build/libSmartServoFramework.dylib $${FW_DIR})
    #QMAKE_POST_LINK += && (if [ ! -d $${FW_DIR}/QtCore.framework/ ]; then cp -R $${QT_DIR}/QtCore.framework $${FW_DIR}; fi)
    #QMAKE_POST_LINK += && (if [ ! -d $${FW_DIR}/QtSvg.framework/ ]; then cp -R $${QT_DIR}/QtSvg.framework $${FW_DIR}; fi)
    #QMAKE_POST_LINK += && (if [ ! -d $${FW_DIR}/QtGui.framework/ ]; then cp -R $${QT_DIR}/QtGui.framework $${FW_DIR}; fi)
    #QMAKE_POST_LINK += && (if [ ! -d $${FW_DIR}/QtWidgets.framework/ ]; then cp -R $${QT_DIR}/QtWidgets.framework $${FW_DIR}; fi)

    # Use bundled libraries (rewrite rpaths)
    APP = build/$${TARGET}.app/Contents/MacOS/$${TARGET}
    #QMAKE_POST_LINK += && (install_name_tool -change $${PWD}/build/libSmartServoFramework.dylib @executable_path/../Frameworks/libSmartServoFramework.dylib $${APP})
    #QMAKE_POST_LINK += && (install_name_tool -change @rpath/QtCore.framework/Versions/5/QtCore @executable_path/../Frameworks/QtCore.framework/Versions/5/QtCore $${APP})
    #QMAKE_POST_LINK += && (install_name_tool -change @rpath/QtSvg.framework/Versions/5/QtSvg @executable_path/../Frameworks/QtSvg.framework/Versions/5/QtSvg $${APP})
    #QMAKE_POST_LINK += && (install_name_tool -change @rpath/QtGui.framework/Versions/5/QtGui @executable_path/../Frameworks/QtGui.framework/Versions/5/QtGui $${APP})
    #QMAKE_POST_LINK += && (install_name_tool -change @rpath/QtWidgets.framework/Versions/5/QtWidgets @executable_path/../Frameworks/QtWidgets.framework/Versions/5/QtWidgets $${APP})
}
