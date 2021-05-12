/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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
*******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#include "port_handler.h"
#include "port_handler_linux.h"

using namespace dynamixel;

std::shared_ptr<PortHandler> PortHandler::getPortHandler(const char *port_name)
{
    return std::dynamic_pointer_cast<PortHandler>(std::make_shared<PortHandlerLinux>(port_name));
}
