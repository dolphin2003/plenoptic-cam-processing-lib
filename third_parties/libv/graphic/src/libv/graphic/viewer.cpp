
/**

\file
Viewer class definition.
\copyright 2013 Institut Pascal

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <QApplication>
#include <QFileDialog>
#include <QFont>
#include <QGLWidget>
#include <QMenu>
#include <QPrinter>
#include <QScrollBar>
#include <QSvgGenerator>
#include <QToolButton>
#include <cmath>

#include "private/global.hpp"
#include "private/viewer_data.hpp"

namespace v {
namespace graphic {
namespace {

using namespace viewers_;

static void init_menu(QToolBar *toolbar, QAction *action)
{
  action->setMenu(new QMenu);
  static_cast<QToolButton *>(toolbar->widgetForAction(action))->setPopupMode(QToolButton::InstantPopup);
}

}

/// Initialize this viewer.
Viewer::Viewer(QWidget *parent)
  : QAbstractScrollArea(parent)
  , that(new ViewerData(this))
{
  that->setupUi(this);
  that->toolbar->addWidget(that->coordinates);
  that->dock->setTitleBarWidget(that->titlebar);
  that->dock->setParent(0);
  that->window->setParent(0);

  setViewport(that->viewport);

  init_menu(that->toolbar, that->menu_visibility);
  init_menu(that->toolbar, that->menu_options);

  QAction *actions[] = {
    that->action_interaction_mode_2d,
    that->action_interaction_mode_cad,
    that->action_interaction_mode_fps,
  };

  for(size_t i = 0; i < 3; ++i)
  {
    menu()->addAction(actions[i]);
    actions[i]->setActionGroup(&that->controllers);
    actions[i]->setData(QVariant::fromValue(i));
    actions[i]->setChecked(!i);
  }
}

Viewer::~Viewer(void)
{
  delete that;
}

/**

Put this viewer in a QMainWindow.

\return  A window containing this viewer.

*/
QMainWindow *Viewer::as_window()
{
  that->window->setCentralWidget(this);
  that->window->addToolBar(that->toolbar);
  that->window->show();
  return that->window;
}

/**

Put this viewer in a QDockWidget.

\return  A dock containing this viewer.

*/
QDockWidget *Viewer::as_dock()
{
  that->dock->setWidget(this);
  that->dock_layout->insertWidget(1, that->toolbar);
  that->window->hide();
  return that->dock;
}

/**

The main menu for this viewer.

\return A menu.

*/
QMenu *Viewer::menu()
{
  return that->menu_options->menu();
}

/// The visibility of the axes.
void
Viewer::axes_are_visible(bool value)
{
  if(!value)
  {
    that->action_show_grid->setChecked(false);
  }
  that->axes_enabled = value;
  that->update();
}

/// The visibility of the grid.
void
Viewer::grid_is_visible(bool value)
{
  if(value)
  {
    that->action_show_axes->setChecked(true);
  }
  that->grid_enabled = value;
  that->update();
}

/// The visibility of the reticle.
bool
Viewer::reticle_is_visible(void) const
{
  return that->action_show_reticle->isChecked();
}

/// The visibility of the reticle.
void
Viewer::reticle_is_visible(bool value)
{
  if(value)
  {
    reticle_pos(QPoint(viewport()->width(), viewport()->height()) / 2);
  }
  else
  {
    that->coordinates->setText(QString());
  }
  that->reticle_enabled = value;
  that->update();
}

/// The position of the reticle.
QPoint
Viewer::reticle_pos(void) const
{
  return that->reticle_pos;
}

/// The position of the reticle.
void
Viewer::reticle_pos(const QPoint &new_pos)
{
  that->reticle_pos = new_pos;
  QPointF p = to_scene(new_pos);
  that->coordinates->setText(QString("%1, %2").arg(p.x()).arg(that->axes_type == AxesXY ? -p.y() : p.y()));
  that->update();
}

/// Adjust the scene so that it fits in the viewport.
void
Viewer::zoom_fit_best(void)
{
  const QRectF rect = bounds();
  const qreal s = std::min(width() / rect.width(), height() / rect.height());
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  that->transform.reset();
  that->transform *= QTransform::fromTranslate(-rect.x(), -rect.y());
  that->transform *= QTransform::fromScale(s, s);
  that->update();
}

/// Adjust the viewport so that the scene fits in it.
void
Viewer::zoom_original(void)
{
  const QRectF rect = bounds();
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  that->transform.reset();
  that->transform *= QTransform::fromTranslate(-rect.x(), -rect.y());
  window()->resize(window()->size() - size() + rect.size().toSize());
  that->has_size = true;
  that->update();
}

void Viewer::zoom(float zoom)
{
  that->transform *= QTransform::fromTranslate(-width() / 2, -height() / 2);
  that->transform *= QTransform::fromScale(zoom / that->transform.m11(), zoom / that->transform.m11());
  that->transform *= QTransform::fromTranslate(width() / 2, height() / 2);
  that->update();
}

void Viewer::look_at(QPointF point)
{