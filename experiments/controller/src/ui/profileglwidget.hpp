/*
 * painter.hpp
 *
 *  Created on: Mar 7, 2019
 *      Author: rob
 */

#include <vector>
#include <mutex>

#include <QtWidgets/QOpenGLWidget>
#include <QtGui/QPainter>

#include "ui/drawconfig.hpp"

class ProfileGLWidget : public QOpenGLWidget {
private:
	QPainter p;
	float minx, miny, maxx, maxy;
	bool boundsSet;

	void calcBounds() {
		miny = minx = std::numeric_limits<float>::max();
		maxy = maxx = std::numeric_limits<float>::lowest();
		for(DrawConfig* config : configs) {
			std::lock_guard<std::mutex> lk(config->mtx);
			for(const auto& it : config->data()) {
				if(it.first < minx) minx = it.first;
				if(it.first > maxx) maxx = it.first;
				if(it.second < miny) miny = it.second;
				if(it.second > maxy) maxy = it.second;
			}
		}
	}

public:
	std::vector<DrawConfig*> configs;

	ProfileGLWidget(QWidget* parent) :
		QOpenGLWidget(parent),
		minx(0), miny(0), maxx(0), maxy(0), boundsSet(false) {}

	void setBounds(double minx, double miny, double maxx, double maxy) {
		this->minx = minx;
		this->maxx = maxx;
		this->miny = miny;
		this->maxy = maxy;
		boundsSet = true;
	}

	void paintGL() {

		if(!boundsSet)
			return;

		int buf = 5;
		float dw = maxx - minx;
		float dh = maxy - miny;

		if(dw > 0 && dh > 0) {

			QPen pen;

			QSize size = this->size();
			int w = size.width() - buf * 2;
			int hh = size.height();
			int h = hh - buf * 2;

			float scale = std::min(w / dw, h / dh);

			p.begin(this);
			for(DrawConfig* config : configs) {
				std::vector<QPointF> pts;
				{
					std::lock_guard<std::mutex> lk(config->mtx);
					const std::vector<std::pair<double, double>>& data = config->data();
					for(const auto& it : data)
						pts.emplace_back(buf + (it.first - minx) * scale, hh - hh / 2 + dh / 2 * scale - (it.second - miny) * scale);
				}
				switch(config->drawType) {
				case DrawType::Line:
					pen.setWidth(1);
					pen.setColor(config->lineColor);
					pen.setStyle(config->lineStyle);
					p.setPen(pen);
					p.drawPolyline(pts.data(), pts.size());
					break;
				case DrawType::Points:
					pen.setWidth(2);
					pen.setColor(config->lineColor);
					pen.setStyle(config->lineStyle);
					p.setPen(pen);
					p.drawPoints(pts.data(), pts.size());
					break;
				case DrawType::Cross:
					pen.setWidth(1);
					pen.setColor(config->lineColor);
					pen.setStyle(config->lineStyle);
					p.setPen(pen);
					for(const QPointF& pt : pts) {
						p.drawLine(pt.x() - 3, pt.y(), pt.x() + 3, pt.y());
						p.drawLine(pt.x(), pt.y() - 3, pt.x(), pt.y() + 3);
					}
					break;
				}
				pts.clear();
			}
			p.end();
		}

	}
};


