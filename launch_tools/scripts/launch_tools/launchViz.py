#!/usr/bin/env python
"""
Copyright (c) 2012,
Systems, Robotics and Vision Group
University of the Balearican Islands
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of
      the Balearican Islands nor the names of its contributors may be used to
      endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


try:
    import os
    import sys
    #import subprocess
    import argparse
    import gtk
    import xdot
    import StringIO
    import fnmatch
    from xml.etree import ElementTree
    import roslib.packages
except ImportError:
    # Checks the installation of the necessary python modules
    print((os.linesep * 2).join(["An error found importing one module:",
    str(sys.exc_info()[1]), "You need to install it", "Stopping..."]))
    sys.exit(-2)

def find_launchfiles(directory, pattern='*.launch'):
    for root, dirs, files in os.walk(directory):
        for basename in files:
            if fnmatch.fnmatch(basename, pattern):
                filename = os.path.join(root, basename)
                yield root, basename, filename

def check_launchfiles(directory, pattern='*.launch'):
    for root, dirs, files in os.walk(directory):
        for basename in files:
            if fnmatch.fnmatch(basename, pattern):
                return True;
    return False;

def prepare_file():
    g = StringIO.StringIO()
    g.write('digraph G {\n rankdir=LR \n')
    return g

def show_graph(g,package = 0):
    drawable = False
    g.write('}')
    if len(g.getvalue())>26:
        drawable = True
        window = xdot.DotWindow()
        window.set_dotcode(g.getvalue())
        if package:
            window.set_title('Package {0}'.format(package))
        window.connect('destroy', gtk.main_quit)
    g.close()
    return drawable

def start_subgraph(g,boxname):
    g.write('  subgraph "{0}"'.format(boxname))
    g.write(' {\n')
    g.write('    label = "{0}";\n'.format(boxname))

def write_information(g,node_name,node_args=0):
    if node_args==0:
        for name in node_name:
            g.write('"{0}_node" [shape=Mrecord,label="{0}",style="filled",fillcolor="khaki"]\n'.format(name))
    else:
        g.write('"{0}" [shape=none, margin=0, label=<\n'.format(node_name))
        g.write('<table border="0" cellborder="1" cellpadding="4">\n')
        g.write('<tr><td BGCOLOR="greenyellow"><b>{0}</b></td></tr>\n'.format(node_name))
        if node_args:
            g.write('<tr><td BGCOLOR="lemonchiffon">Arguments:</td></tr>\n')
            for (arg, arg_def) in node_args:
                g.write('<tr><td ALIGN="LEFT" BGCOLOR="lemonchiffon">{0} = {1}</td></tr>\n'.format(arg,arg_def))
        g.write('</table>>];\n')

def write_connection(g,origin,destination, isNode=False):
    if not isNode:
        g.write('    "{0}" -> "{1}"\n'.format(origin,destination))
    elif isNode:
        g.write('    "{0}" -> "{1}_node"\n'.format(origin,destination))

def get_args_from_parser():
    parser = argparse.ArgumentParser(description='Draws graph from launchfiles')
    parser.add_argument('--pkg', metavar='package', help='ROS packages you want to inspect',nargs='+')
    return parser.parse_args()

def draw_folder(g,folder):
    working_directory = folder
    if check_launchfiles(folder):
        for root,filename,filepath in find_launchfiles(folder):
            if root != working_directory:
                working_directory = root
            f = open(filepath,'rt')
            try:
                tree = ElementTree.parse(f)
                node_name = os.path.splitext(filename)[0] #remove extension
                it = tree.iter()
                #Iterate
                node_args = []
                node_includes = []
                node_nodes = []
                for node in it:
                    if node.tag == "arg":
                        #save arguments and default values
                        arg_name = node.attrib.get('name')
                        default_value = node.attrib.get('default')
                        value = node.attrib.get('value')
                        if_clause = node.attrib.get('if')
                        if default_value:
                            #add to the node_args list with default
                            node_args.append((arg_name,default_value))
                        elif not value:
                            #add to the node_args list with REQUIRED
                            node_args.append((arg_name,"REQUIRED"))
                        elif if_clause:
                            nodelet_args = value.split()[1]
                    if node.tag == "include":
                        #remove extension and path to file
                        incl_ext = os.path.basename(node.attrib.get('file'))
                        incl = os.path.splitext(incl_ext)[0]
                        node_includes.append(incl)
                    if node.tag == "node":
                        node_type = node.attrib.get('type')
                        if node_type == "nodelet":
                            node_type = node.attrib.get('args')
                            if len(node_type.split())>1:
                                node_type = node_type.split()[1]
                            if node_type[len(node_type)-1] == ")":
                                node_type = nodelet_args
                            node_nodes.append(node_type)
                        else:
                            node_nodes.append(node_type)

                write_information(g,node_name,node_args)
                write_information(g,node_nodes)
                for dest in node_nodes:
                    write_connection(g,node_name,dest,True)
                for dest in node_includes:
                    write_connection(g,node_name,dest)
            except ElementTree.ParseError:
                rospy.loginfo("Error parsing {}".format(filepath))
        return g
    else:
        rospy.loginfo("[W]\tNo launchfile was found in {0}".format(os.path.basename(folder)))

def xml_process(packages = 0):
    g = prepare_file()
    if not packages:
        working_directory = os.getcwd();
        draw_folder(g,working_directory)
    else:
        for pkg in packages:
          directory = roslib.packages.get_pkg_dir(pkg)
          draw_folder(g,directory)
    return g

def main():
    args = get_args_from_parser()
    g = xml_process(args.pkg)
    if show_graph(g):
      gtk.main()

if __name__ == "__main__":
    main()
