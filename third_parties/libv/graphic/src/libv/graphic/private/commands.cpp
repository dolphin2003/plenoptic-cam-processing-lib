/**

\file
\author Alexis Wilhelm (2013)
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

#include <QMenu>
#include <memory>
#include <cmath>
#include <boost/preprocessor.hpp>

#include "global.hpp"
#include "viewer_data.hpp"

namespace v {
namespace graphic {
namespace {

using namespace viewers_;

#define _DEFINE_COMMAND(_name, _args, _value)\
  _DEFINE_COMMAND_OVERLOAD(_name, _args, _value)\
  _DEFINE_COMMAND_IMPLEMENTATION(_name)

#define _DEFINE_COMMAND_IMPLEMENTATION(_name)\
  void Commands::_name::operator()(Viewer *viewer)

#define _DEFINE_COMMAND_OVERLOAD(_name, _args, _value)\
\
  ViewerContext &\
  ViewerContext::_name _args\
  {\
    std::lock_guard<std::mutex> lock(mutex_);\
    _COMMANDS.push_back(CommandList::value_type::create<Command<Commands::_name> >(construct<Commands::_name> _value));\
    return *this;\
  }\

#define _DEFINE_CONSTRUCTOR(z, n, data)\
\
  template<class T, BOOST_PP_ENUM_PARAMS(n, class T)>\
  static T construct(BOOST_PP_ENUM_BINARY_PARAMS(n, const T, &a))\
  {\
    T a = {BOOST_PP_ENUM_PARAMS(n, a)};\
    return a;\
  }\

BOOST_PP_REPEAT_FROM_TO(1, 10, _DEFINE_CONSTRUCTOR, :)

#define _THIS (*viewer)
#define _THAT (*_THIS.that)
#define _LAYER (_THAT.layers[context.layer()])
#define _CONTEXT (_LAYER.data[context])
#define _COMMANDS (commands_[viewer()].first)

#define _(y) (viewer->that->axes_type == AxesXY ? -(y) : (y))

static uint32_t to_int(v::RGBAU8 value)
{
  return qRgba(value.r(), value.g(), value.b(), value.a());
}

static v::ImageBGRAU8 to_bgra(const v::ImageRGBAU8cr &rgba)
{
  v::ImageBGRAU8 bgra(rgba.height(), rgba.width());
  V_FOR_EACH_PIXEL(p, rgba)
  {
    bgra[p.row][p.column].r(p->r()).g(p->g()).b(p->b()).a(p->a());
  }
  return bgra;
}

}

_DEFINE_COMMAND(size, (int width, int height), (width, height))
{
  _THIS.window()->resize(_THIS.window()->size() - _THIS.size() + QSize(width, height));
  _THAT.has_size = true;
}

_DEFINE_COMMAND(look_at, (float x, float y), (x, y))
{
  _THIS.look_at(QPointF(x, _(y)));
}

_DEFINE_COMMAND(zoom, (float zoom), (zoom))
{
  _THIS.zoom(zoom);
}

_DEFINE_COMMAND(title, (const std::string &title), (title))
{
  QString title = QString::fromUtf8(title_.c_str());
  _THIS.setWindowTitle(title);
  _THAT.window->setWindowTitle(title);
  _THAT.dock->setWindowTitle(title);
  _THAT.title->setText(title);
}

_DEFINE_COMMAND(axes, (ViewerAxesType value), (value))
{
  _THAT.axes_type = value;
}

