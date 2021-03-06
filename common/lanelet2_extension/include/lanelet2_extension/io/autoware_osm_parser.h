/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Ryohsuke Mitsudome
 */

#ifndef LANELET2_EXTENSION_IO_AUTOWARE_OSM_PARSER_H
#define LANELET2_EXTENSION_IO_AUTOWARE_OSM_PARSER_H

#include <lanelet2_io/io_handlers/OsmHandler.h>

#include <string>

namespace lanelet
{
namespace io_handlers
{
class AutowareOsmParser : public OsmParser
{
public:
  using OsmParser::OsmParser;

  /**
   * [parse parse osm file to laneletMap. It is generally same as default
   * OsmParser, but it will overwrite x and y value with local_x and local_y
   * tags if present]
   * @param  filename [path to osm file]
   * @param  errors   [any errors catched during parsing]
   * @return          [returns LaneletMap]
   */
  std::unique_ptr<LaneletMap> parse(const std::string& filename, ErrorMessages& errors) const;  // NOLINT

  /**
   * [parseVersions parses MetaInfo tags from osm file]
   * @param filename       [path to osm file]
   * @param format_version [parsed information about map format version]
   * @param map_version    [parsed information about map version]
   */
  static void parseVersions(const std::string& filename, std::string* format_version, std::string* map_version);

  /**
   * [parseMapParams parses GeoReference (i.e. map parameters) info from osm file and loads default ECEF proj strings]
   * NOTE: geoReference is expected to be encoded inside bracket:<geoReference>string</geoReference>, or as "v" or "value"
   *       attributes: <geoReference v="string"/> or <geoReference value="string"/>
   *       projector_type = 0 is currently not supported by CARMA
   *       +geoidgrids is also currently not a supported georeference field by the parser
   * @param filename            [path to osm file]
   * @param projector_type      [parsed information about map projector_type: Currently it supposes 0 as Autoware default MGRSProjector and 1 as CARMA LocalFrameProjector]
   * @param target_frame        [parsed information about map geo reference: target_frame]
   * @throw lanelet::ParseError [throws if either frame is nullptr or if geoReference tag cannot be found in .osm file]
   */
  static void parseMapParams(const std::string& filename, int* projector_type, std::string* target_frame);

  static constexpr const char* extension()
  {
    return ".osm";
  }

  static constexpr const char* name()
  {
    return "autoware_osm_handler";
  }
};

}  // namespace io_handlers
}  // namespace lanelet

#endif  // LANELET2_EXTENSION_IO_AUTOWARE_OSM_PARSER_H
