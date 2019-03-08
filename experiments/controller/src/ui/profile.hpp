#include <unordered_set>

#include <QtWidgets/QDialog>

#include "ui_profile.h"

#include "ui/drawconfig.hpp"

class ProfileDialog : public QDialog, public Ui::ProfileDialog {

	Q_OBJECT

protected:
	std::unordered_set<DrawConfig*> drawConfigs;

public:

	void setupUi(QDialog *dialog);

	void closeClicked();

	void addDrawConfig(DrawConfig* config);

	void removeDrawConfig(DrawConfig* config);

	void draw();

	static ProfileDialog* instance();

};
