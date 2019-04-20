#include <unordered_set>

#include <QtWidgets/QDialog>

#include "ui_profile.h"

#include "ui/drawconfig.hpp"

/**
 * The profile dialog window for displaying point cloud, surface and trajectory.
 */
class ProfileDialog : public QDialog, public Ui::ProfileDialog {

	Q_OBJECT

protected:
	std::unordered_set<DrawConfig*> drawConfigs;

public:

	bool done;

	void setupUi(QDialog *dialog);

	void addDrawConfig(DrawConfig* config);

	void removeDrawConfig(DrawConfig* config);

	void setBounds(double minx, double miny, double maxx, double maxy);

	void draw();

	static ProfileDialog* instance();

public slots:

	void closeClicked();

};
