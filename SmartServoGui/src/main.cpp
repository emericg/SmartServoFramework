/*!
 * This file is part of SmartServoFramework.
 * Copyright (c) 2014, INRIA, All rights reserved.
 *
 * SmartServoFramework is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/lgpl-3.0.txt>.
 *
 * \file main.cpp
 * \date 27/03/2014
 * \author Emeric Grange <emeric.grange@gmail.com>
 */

#include "mainwindow.h"

#include <QApplication>
#include <QLibraryInfo>
#include <QTranslator>
#include <QLocale>
#include "qglobal.h"

int main(int argc, char *argv[])
{
#ifdef Q_OS_WIN32
    // High DPI monitor? Use automatic scaling
    qputenv("QT_AUTO_SCREEN_SCALE_FACTOR", "1");
#endif

    Q_INIT_RESOURCE(resources);
    QApplication app(argc, argv);
    app.setOrganizationName("SmartServoGui");
    app.setApplicationDisplayName("SmartServoGui");
    QCoreApplication::setApplicationName("SmartServoGui");

    // Handle Qt translation
    QTranslator qtTranslator;
    qtTranslator.load("qt_" + QLocale::system().name(), QLibraryInfo::location(QLibraryInfo::TranslationsPath));
    app.installTranslator(&qtTranslator);

    // Handle SmartServoGui translation
    QTranslator appTranslator;
    QString locale = QLocale::system().name().section('_', 0, 0);
    if (appTranslator.load("../resources/lang/" + locale) == true)
    {
        app.installTranslator(&appTranslator);
    }

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    // High DPI monitor? Use high quality pixmaps
    if (app.devicePixelRatio() > 1)
    {
        app.setAttribute(Qt::AA_UseHighDpiPixmaps);
    }
#endif

    // Initialize program window
    MainWindow w;
    w.show();

    // Start initial port scanning
    w.autoScanPorts();

    return app.exec();
}
