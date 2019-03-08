/*
 * painter.hpp
 *
 *  Created on: Mar 7, 2019
 *      Author: rob
 */

#include <unordered_set>

#include <QtWidgets/QOpenGLWidget>
#include <QtGui/QPainter>

#include "ui/drawconfig.hpp"

class ProfileGLWidget : public QOpenGLWidget {
private:
	QPainter p;
	float minx, miny, maxx, maxy;

	void calcBounds() {
		miny = minx = std::numeric_limits<float>::max();
		maxy = maxx = std::numeric_limits<float>::lowest();
		for(const DrawConfig* config : configs) {
			for(const auto& it : config->data) {
				if(it.first < minx) minx = it.first;
				if(it.first > maxx) maxx = it.first;
				if(it.second < miny) miny = it.second;
				if(it.second > maxy) maxy = it.second;
			}
		}
	}

public:
	std::unordered_set<DrawConfig*> configs;

	ProfileGLWidget(QWidget* parent) :
		QOpenGLWidget(parent),
		minx(0), miny(0), maxx(0), maxy(0) {}

	void paintGL() {

		calcBounds();

		QPen pen;

		int buf = 5;

		float dw = maxx - minx;
		float dh = maxy - miny;
		if(dw > 0 && dh > 0) {

			QSize size = this->size();
			int w = size.width() - buf * 2;
			int h = size.height() - buf * 2;

			p.begin(this);
			for(const DrawConfig* config : configs) {
				std::vector<QPointF> pts;
				for(const auto& it : config->data)
					pts.emplace_back(buf + ((it.first - minx) / dw) * w, buf + ((it.second - miny) / dh) * h);
				pen.setColor(config->lineColor);
				pen.setStyle(config->lineStyle);
				p.setPen(pen);
				p.drawPolyline(pts.data(), pts.size());
				pts.clear();
			}
			p.end();
		}

	}
};


