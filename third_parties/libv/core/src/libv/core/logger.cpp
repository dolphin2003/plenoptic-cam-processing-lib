/**

\file
\author Alexis Wilhelm (2015)
\copyright 2015 Institut Pascal

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

#include <chrono>
#include <cstring>
#include <iostream>
#include <map>

#include "logger.hpp"

namespace v {
namespace core {
namespace {

using Time = std::chrono::steady_clock;

struct Tuple
{
  size_t count = 0;
  Time::duration max{0};
  Time::duration total{0};
};

static float to_float(const Time::duration &d)
{
  return float(d.count()) * Time::period::num / Time::period::den;
}

struct Data
: std::map<const char *, Tuple>
{
  ~Data()
  {
    for(auto &&p: *this)
    {
      const float d = to_float(p.second.total);
      Logger<true>{v::log_stderr, p.first}
        << " calls=" << p.second.count
        << " mean=" << d / p.second.count
        << " max=" << to_float(p.second.max)
        << " total=" << d
        ;
    }
  }
};

template<std::ostream &_stream>
static void log_ostream(std::ostringstream *s)
{
  // idéalement, il faudrait verrouiller un mutex ici, mais alors on introduirait une dépendance à une bibliothèque de threads
  // static std::mutex mutex; std::lock_guard<std::mutex> lock(mutex);
  _stream << s->str();
}

}

void log_stdout(std::ostringstream *s) { return log_ostream<std::cout>(s); }
void log_stderr(std::ostringstream *s) { return log_ostream<std::cerr>(s); }

/**

Raccourcit un chemin pour alléger les messages de debug.

Quand on préfixe les messages de debug par le chemin vers le fichier d'où ils viennent, ça peut faire un affichage très lourd si le chemin est un peu long.
Pour alléger l'affichage, on raccourcit ce chemin en ne gardant que le nom du fichier.

*/
const char *shorten_file_name(const cha