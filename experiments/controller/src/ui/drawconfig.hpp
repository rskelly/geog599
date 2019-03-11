/*
 * drawconfig.hpp
 *
 *  Created on: Mar 7, 2019
 *      Author: rob
 */

#ifndef SRC_UI_DRAWCONFIG_HPP_
#define SRC_UI_DRAWCONFIG_HPP_

#include <unordered_set>

#include <QtWidgets/QDialog>

enum DrawType {
	Line,
	Points,
	Cross
};

class DrawConfig {
public:
	Qt::PenStyle lineStyle;
	QColor lineColor;
	Qt::BrushStyle fillStyle;
	QColor fillColor;
	DrawType drawType;
	std::vector<std::pair<double, double>> data;

	DrawConfig() {
		lineStyle = Qt::PenStyle::SolidLine;
		lineColor = QColor::fromRgb(255, 0, 0, 255);
		fillStyle = Qt::BrushStyle::SolidPattern;
		fillColor = QColor::fromRgb(0, 255, 0, 255);
		drawType = DrawType::Line;
	}

	void setType(DrawType type) {
		drawType = type;
	}

	void setFillColor(int r, int g, int b) {
		fillColor = QColor::fromRgb(r, g, b, 255);
	}

	void setLineColor(int r, int g, int b) {
		lineColor = QColor::fromRgb(r, g, b, 255);
	}

};


#endif /* SRC_UI_DRAWCONFIG_HPP_ */
