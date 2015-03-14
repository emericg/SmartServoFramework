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
#include <QTranslator>
#include <QLibraryInfo>

int main(int argc, char *argv[])
{
    Q_INIT_RESOURCE(resources);
    QApplication app(argc, argv);

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

    // Initialize program window
    MainWindow w;
    w.show();

    // Start initial "automatic" scanning
    w.scanSerialPorts();

    return app.exec();
}
