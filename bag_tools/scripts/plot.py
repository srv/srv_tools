#!/usr/bin/python
"""
Copyright (c) 2015,
Enrique Fernandez Perdomo
Clearpath Robotics, Inc.
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

# @todo split all this into libraries
from __future__ import print_function

import roslib
import roslib.msgs
import rosbag
import rosgraph

import sys
import os.path
import numpy
import argparse
import string

from operator import itemgetter

# Workaround to avoid issues with X11 rendering when running on background:
import matplotlib as mpl
mpl.use('Agg')

import matplotlib.pyplot as plt


def get_nested_attribute(msg, nested_attributes):
    """
    Get nested attribute

    This function has been taken from rostopic.__init__.py

    :param msg: msg object
    :param nested_attributes: nested attributes on the object, ``str``
    :returns: nested attribute
    """
    value = msg
    for attr in nested_attributes.split('/'):
        value = getattr(value, attr)
    return value


def get_array_index_or_slice_object(index_string):
    """
    Get array index or slice object.

    This function has been taken from rostopic.__init__.py

    :param index_string: index string, ``str``
    :returns: array index or slice object
    """
    assert index_string != '', 'empty array index'

    index_string_parts = index_string.split(':')
    if len(index_string_parts) == 1:
        try:
            array_index = int(index_string_parts[0])
        except ValueError:
            assert False, "non-integer array index step '%s'" % index_string_parts[0]
        return array_index

    slice_args = [None, None, None]
    if index_string_parts[0] != '':
        try:
            slice_args[0] = int(index_string_parts[0])
        except ValueError:
            assert False, "non-integer slice start '%s'" % index_string_parts[0]

    if index_string_parts[1] != '':
        try:
            slice_args[1] = int(index_string_parts[1])
        except ValueError:
            assert False, "non-integer slice stop '%s'" % index_string_parts[1]

    if len(index_string_parts) > 2 and index_string_parts[2] != '':
            try:
                slice_args[2] = int(index_string_parts[2])
            except ValueError:
                assert False, "non-integer slice step '%s'" % index_string_parts[2]
    if len(index_string_parts) > 3:
        assert False, 'too many slice arguments'

    return slice(*slice_args)


def msgevalgen(pattern):
    """
    Generates a function that returns the relevant field (aka 'subtopic') of a
    Message object.

    This functions is taken from rostopic.__init__.py

    :param pattern: subtopic, e.g. /x. Must have a leading '/' if specified,
    ``str``
    :returns: function that converts a message into the desired value,
    ``fn(Message) -> value``
    """
    if not pattern or pattern == '/':
        return None

    assert pattern[0] == '/'
    msg_attribute = pattern[1:]

    # Use slice arguments if present:
    array_index_or_slice_object = None
    index = msg_attribute.find('[')
    if index != -1:
        if not msg_attribute.endswith(']'):
            print("Topic name '%s' contains '[' but does not end with ']'\n" % msg_attribute, file=sys.stderr)
            return None

        index_string = msg_attribute[index + 1:-1]
        try:
            array_index_or_slice_object = get_array_index_or_slice_object(index_string)
        except AssertionError as e:
            print("Topic name '%s' contains invalid slice argument '%s': %s\n" % (msg_attribute, index_string, str(e)), file=sys.stderr)
            return None

        msg_attribute = msg_attribute[:index]

    def msgeval(msg):
        # This can be replaced with something less beautiful but more efficient
        try:
            value = get_nested_attribute(msg, msg_attribute)
        except AttributeError:
            print("no field named [%s]" % pattern, file=sys.stderr)
            return None

        if array_index_or_slice_object is not None:
            value = value[array_index_or_slice_object]

        return value

    return msgeval


class BagTopicHelper:
    """
    Bag topic helper, which allows to retrieve information about the topics on a
    bag file and their corresponding types.
    This class is similar in functionality to the functions provided in
    rqt_py_common.topic_helpers.py, but those operate on the topics registered on
    the ROS master instead of a bag file.
    """

    def __init__(self, bag):
        self._bag = bag
        self._error = False

        self._topic_types = None

    def get_topic_types(self):
        """
        Retrieves the type and topic info and cached them.
        The first time it opens the bag file and then closes it.
        If there's any error opening or reading the bag file, the error flag is
        set to True.

        :returns: array of (topic, type) tuples
        """
        if self._topic_types is None and not self._error:
            try:
                bag_file = rosbag.Bag(self._bag, 'r')

                info = bag_file.get_type_and_topic_info()[1]
                self._topic_types = [(key, value.msg_type) for key, value in info.iteritems()]

                bag_file.close()
            except (IOError, rosbag.ROSBagException):
                self._error = True

        return self._topic_types

    # @todo see if the fn returned here can replace the ROSData class
    def get_topic_type(self, topic):
        """
        Get the topic type.

        This method is based on the homonymous function provided in
        rostopic.__init__.py (actually the private _get_topic_type version)

        :param topic: topic name which can optionally include a field, ``str``
        :returns: topic type, real topic name, fields and fn to evaluate the
        message instance if the topic points to a field withing a topic,
        ``(str, str, str, fn)``
        """
        # Get topic types:
        topic_types = self.get_topic_types()

        if topic_types is None:
            return None, None, None, None

        # Exact match first, followed by prefix match:
        matches = [(t, t_type) for t, t_type in topic_types if t == topic]
        if not matches:
            matches = [(t, t_type) for t, t_type in topic_types if topic.startswith(t+'/')]
            # Choose longest match:
            matches.sort(key=itemgetter(0), reverse=True)

            # Try to ignore messages which don't have the field specified as
            # part of the topic name:
            while matches:
                t, t_type = matches[0]
                msg_class = roslib.message.get_message_class(t_type)
                if not msg_class:
                    # If any class is not fetchable skip ignoring any message
                    # types
                    break
                msg = msg_class()
                nested_attributes = topic[len(t) + 1:].rstrip('/')
                nested_attributes = nested_attributes.split('[')[0]
                if nested_attributes == '':
                    break
                try:
                    get_nested_attribute(msg, nested_attributes)
                except AttributeError:
                    # Ignore this type since it does not have the requested
                    # field:
                    matches.pop(0)
                    continue
                matches = [(t, t_type)]
                break

        if matches:
            t, t_type = matches[0]
            if t_type == rosgraph.names.ANYTYPE:
                return None, None, None, None
            return t_type, t, topic[len(t):], msgevalgen(topic[len(t):])
        else:
            return None, None, None, None

    def get_field_type(self, topic_name):
        """
        Get the Python type of a specific field in the given registered topic.
        If the field is an array, the type of the array's values are returned and
        the is_array flag is set to True.

        This method is based on the homonymous function provided in
        rqt_py_common.topic_helpers.py

        :param topic_name: name of the field of a bag topic, ``str``
        :returns: field_type, is_array
        """
        # Get topic type and message evaluator:
        topic_type, real_topic_name, _, _ = self.get_topic_type(topic_name)
        if topic_type is None:
            return None, False

        message_class = roslib.message.get_message_class(topic_type)
        if message_class is None:
            return None, False

        slot_path = topic_name[len(real_topic_name)]
        return self.get_slot_type(message_class, slot_path)

    def get_slot_type(self, message_class, slot_path):
        """
        Get the Python type of a specific slot in the given message class.
        If the field is an array, the type of the array's values are returned and
        the is_array flag is set to True.

        This method is based on the homonymous function provided in
        rqt_py_common.topic_helpers.py

        :param message_class: message class type, ``type``, usually inherits from
        genpy.message.Message
        :param slot_path: path to the slot inside the message class, ``str``
        :returns: field_type, is_array
        """
        is_array = False
        fields = [f for f in slot_path.split('/') if f]
        for field_name in fields:
            try:
                field_name, _, field_index = roslib.msgs.parse_type(field_name)
            except roslib.msgs.MsgSpecException:
                return None, False

            if field_name not in getattr(message_class, '__slots__', []):
                return None, False
            slot_type = message_class._slot_types[message_class.__slots__.index(field_name)]
            slot_type, slot_is_array, _ = roslib.msgs.parse_type(slot_type)
            is_array = slot_is_array and field_index is None

            message_class = self.get_type_class(slot_type)

        return message_class, is_array

    def get_type_class(self, type_name):
        """
        Get the Python type of a specific type name.

        This method is based on the homonymous function provided in
        rqt_py_common.topic_helpers.py

        :param type_name: type name, ``str``
        :returns: type
        """
        if roslib.msgs.is_valid_constant_type(type_name):
            if type_name == 'string':
                return str
            elif type_name == 'bool':
                return bool
            else:
                return type(roslib.msgs._convert_val(type_name, 0))
        else:
            return roslib.message.get_message_class(type_name)

    def is_slot_numeric(self, topic_name):
        """
        Check if a slot in the given topic is numeric, or an array of numeric
        values.

        This method is based on the homonymous function provided in
        rqt_py_common.topic_helpers.py

        :param topic_name: name of field of a registered topic, ``str``
        :returns: is_numeric, is_array, description
        """
        field_type, is_array = self.get_field_type(topic_name)
        if field_type in (int, float):
            if is_array:
                message = 'topic "%s" is numeric array: %s[]' % (topic_name, field_type)
            else:
                message = 'topic "%s" is numeric: %s' % (topic_name, field_type)
            return True, is_array, message

        return False, is_array, 'topic "%s" is NOT numeric: %s' % (topic_name, field_type)


def _array_eval(field_name, slot_num):
    """
    :param field_name: name of field to index into, ``str``
    :param slot_num: index of slot to return, ``str``
    :returns: fn(msg_field)->msg_field[slot_num]
    """
    def fn(f):
        return getattr(f, field_name).__getitem__(slot_num)
    return fn


def _field_eval(field_name):
    """
    :param field_name: name of field to return, ``str``
    :returns: fn(msg_field)->msg_field.field_name
    """
    def fn(f):
        return getattr(f, field_name)
    return fn


def generate_field_evals(fields):
    try:
        evals = []
        fields = [f for f in fields.split('/') if f]
        for f in fields:
            if '[' in f:
                field_name, rest = f.split('[')
                slot_num = string.atoi(rest[:rest.find(']')])
                evals.append(_array_eval(field_name, slot_num))
            else:
                evals.append(_field_eval(f))
        return evals
    except Exception:
        return None


class ROSData:
    def __init__(self, topic, bag_topic_helper):
        topic_type, real_topic, fields, _ = bag_topic_helper.get_topic_type(topic)
        if topic_type is not None:
            self._field_evals = generate_field_evals(fields)

    def get_data(self, msg):
        val = msg
        try:
            if not self._field_evals:
                return float(val)

            for f in self._field_evals:
                val = f(val)

            return float(val)
        except (IndexError, TypeError):
            return None


class BagTopicPlotter:
    def __init__(self, bags, topics, plot_arrays=True, plot_headers=True, plot_format='png', plot_style='.-', bag_time=False, out_suffix = ''):
        self._bags = bags
        self._plot_arrays = plot_arrays
        self._plot_headers = plot_headers
        self._plot_format = plot_format
        self._plot_style = plot_style
        self._bag_time = bag_time
        self._out_suffix = out_suffix

        self._bag_topic_helpers = {}
        for bag in self._bags:
            self._bag_topic_helpers[bag] = BagTopicHelper(bag)

        # @todo support multiple bags, for now we take the first one only
        if len(self._bags) > 1:
            print("Does NOT support multiple bags yet! Only first bag will be used.", file=sys.stderr)
        bag = self._bags[0]

        print("Processing %s ..." % bag)

        # Retrieve/Expand plottable topic fields:
        plottable_topics = []
        for topic in topics:
            plottable_topics += self.get_plot_fields(topic)[0]

        if len(plottable_topics) == 0:
            print("No topic/field can be plotted!", file=sys.stderr)
            return

        # Create topic evaluators:
        topic_evaluators = {}
        for topic in plottable_topics:
            _, real_topic, _, _ = self._bag_topic_helpers[bag].get_topic_type(topic)

            if not topic_evaluators.has_key(real_topic):
                topic_evaluators[real_topic] = {}

            if not topic_evaluators[real_topic].has_key(topic):
                evaluator = ROSData(topic, self._bag_topic_helpers[bag])
                topic_evaluators[real_topic][topic] = evaluator

        # Create lists of values for each topic:
        values = {}
        times = {}
        for real_topic in topic_evaluators.iterkeys():
            for topic in topic_evaluators[real_topic].iterkeys():
                values[topic] = []
                times[topic] = []

        # Evaluate messages on the bag file for the topics:
        try:
            bag_file = rosbag.Bag(bag, 'r')

            for real_topic, msg, t in bag_file.read_messages(topics=topic_evaluators.keys()):
                value = {}
                for topic in topic_evaluators[real_topic].iterkeys():
                    value = topic_evaluators[real_topic][topic].get_data(msg)
                    values[topic].append(value)

                    if self._bag_time or not msg._has_header:
                        time = t
                    else:
                        time = msg.header.stamp

                    times[topic].append(time.to_sec())
        except IOError as e:
            print('Failed to open bag file %s: %s!' % (bag, e.strerror), file=sys.stderr)
            return
        except rosbag.ROSBagException as e:
            print('Failed to read bag file %s: %s!' % (bag, e.message), file=sys.stderr)
            return

        min_time = []
        for key in times.iterkeys():
            min_time.append(times[key][0])

        min_time = numpy.min(min_time)

        for key in times.iterkeys():
            key_times = numpy.array(times[key])
            key_times -= min_time
            times[key] = key_times.tolist()

        try:
            fig = plt.figure()
            fig.set_size_inches(20, 15)

            plt.title(os.path.basename(os.path.splitext(bag)[0]))
            plt.xlabel('time [s]')
            plt.grid(True)

            ax = plt.axes()
            ax.set_color_cycle([plt.cm.rainbow(i) for i in numpy.linspace(0, 1, len(values))])

            for topic in plottable_topics:
                plt.plot(times[topic], values[topic], self._plot_style)

            plt.legend(plottable_topics, loc=0)

            plt.savefig(bag.replace('.bag', '') + self._out_suffix + '.' + self._plot_format)

            plt.close(fig)
        except OverflowError as e:
            print('Failed to save plot as %s image file (try other format, e.g. svg): %s', self._plot_format, e.message, file=sys.stderr)

    def get_plot_fields(self, topic_name):
        """
        Get plot fields for a topic name.

        This methods is based on the homonymous one in rqt_plot.plot_widget.py

        :param topic_name: topic name, ``str``
        :returns: plottable fields
        """
        topic_type, real_topic, _, _ = self._bag_topic_helpers[self._bags[0]].get_topic_type(topic_name)
        if topic_type is None:
            message = "topic %s does not exist" % ( topic_name )
            return [], message
        field_name = topic_name[len(real_topic)+1:]

        slot_type, is_array, array_size = roslib.msgs.parse_type(topic_type)
        field_class = roslib.message.get_message_class(slot_type)

        fields = [f for f in field_name.split('/') if f]

        for field in fields:
            # Parse the field name for an array index:
            try:
                field, _, field_index = roslib.msgs.parse_type(field)
            except roslib.msgs.MsgSpecException:
                message = "invalid field %s in topic %s" % ( field, real_topic )
                return [], message

            if field not in getattr(field_class, '__slots__', []):
                message = "no field %s in topic %s" % ( field_name, real_topic )
                return [], message

            slot_type = field_class._slot_types[field_class.__slots__.index(field)]
            slot_type, slot_is_array, array_size = roslib.msgs.parse_type(slot_type)
            is_array = slot_is_array and field_index is None

            field_class = self._bag_topic_helpers[self._bags[0]].get_type_class(slot_type)

        if field_class in (bool, int, float):
            if is_array:
                if array_size is not None:
                    if self._plot_arrays:
                        message = "topic %s is fixed-size numeric array" % ( topic_name )
                        return [ "%s[%d]" % (topic_name, i) for i in range(array_size) ], message
                    else:
                        message = "topic %s is fixed-size numeric array, but plotting arrays is disabled" % ( topic_name )
                        return [], message
                else:
                    message = "topic %s is variable-size numeric array" % ( topic_name )
                    return [], message
            else:
                message = "topic %s is numeric" % ( topic_name )
                return [ topic_name ], message
        else:
            if not roslib.msgs.is_valid_constant_type(slot_type):
                numeric_fields = []
                for i, slot in enumerate(field_class.__slots__):
                    slot_type = field_class._slot_types[i]
                    slot_type, is_array, array_size = roslib.msgs.parse_type(slot_type)
                    slot_class = self._bag_topic_helpers[self._bags[0]].get_type_class(slot_type)
                    if slot_class in (bool, int, float) and not is_array:
                        numeric_fields.append(slot)
                    elif slot != 'header' or self._plot_headers:
                        slot_topics, _ = self.get_plot_fields("%s/%s" % (topic_name, slot))
                        for slot_topic in slot_topics:
                            numeric_fields.append(slot_topic.replace(topic_name + '/', ''))

                message = ""
                if len(numeric_fields) > 0:
                    message = "%d plottable fields in %s" % ( len(numeric_fields), topic_name )
                else:
                    message = "No plottable fields in %s" % ( topic_name )
                return [ "%s/%s" % (topic_name, f) for f in numeric_fields ], message
            else:
                message = "Topic %s is not numeric" % ( topic_name )
                return [], message


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Plots a list of topics into a single axis. '
                    'All the numeric fields are plotted in different series. '
                    'A field of the topic can also be given, in which case '
                    'all its sub-fields are plotted (if any).')

    parser.add_argument('--bags', help='input bagfile(s)', nargs='+')
    # @todo separate with comma ',' the topics that should go to different plots
    # @todo if --topics is '/' all topics and fields should be plotted
    # @todo make --topics optional and default to '/'
    # @todo generate all topics for a given namespace, so all the topics in it
    # are plotted (with all their sub-fields)
    parser.add_argument('--topics', help='input topic(s)', nargs='+')
    parser.add_argument('--noarr', help='do not plot array fields', action='store_true')
    parser.add_argument('--nohdr', help='do not plot header fields', action='store_true')
    parser.add_argument('--plot_format', help='output plot format', default='png')
    parser.add_argument('--plot_style', help='output plot style', default='.-')
    parser.add_argument('--bag_time', help='use bag time instead of msg header time', action='store_true')
    parser.add_argument('--out_suffix', help='output suffix', default='')

    args = parser.parse_args()

    try:
        BagTopicPlotter(args.bags, args.topics, not args.noarr, not args.nohdr, args.plot_format, args.plot_style, args.bag_time, args.out_suffix)
    except Exception, e:
        import traceback
        traceback.print_exc()
