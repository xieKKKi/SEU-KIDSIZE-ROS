# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

from __future__ import print_function

"""
Data structures and library for representing ROS Computation Graph state.
"""

import sys
import time
import itertools
import random
import logging
import traceback
import socket
from roslibpy import Ros


logger = logging.getLogger('rosgraph.graph')

_ROS_NAME = '/rosviz'


def topic_node(topic):
    """
    In order to prevent topic/node name aliasing, we have to remap
    topic node names. Currently we just prepend a space, which is
    an illegal ROS name and thus not aliased.
    @return str: topic mapped to a graph node name.
    """
    return ' ' + topic


def node_topic(node):
    """
    Inverse of topic_node
    @return str: undo topic_node() operation
    """
    return node[1:]


class BadNode(object):
    """
    Data structure for storing info about a 'bad' node
    """

    ## no connectivity
    DEAD = 0
    ## intermittent connectivity
    WONKY = 1

    def __init__(self, name, type, reason):
        """
        @param type: DEAD | WONKY
        @type  type: int
        """
        self.name = name
        self.reason = reason
        self.type = type


class EdgeList(object):
    """
    Data structure for storing Edge instances
    """
    __slots__ = ['edges_by_start', 'edges_by_end']

    def __init__(self):
        # in order to make it easy to purge edges, we double-index them
        self.edges_by_start = {}
        self.edges_by_end = {}

    def __iter__(self):
        return itertools.chain(*[v for v in self.edges_by_start.values()])

    def has(self, edge):
        return edge in self

    def __contains__(self, edge):
        """
        @return: True if edge is in edge list
        @rtype: bool
        """
        key = edge.key
        return key in self.edges_by_start and \
               edge in self.edges_by_start[key]

    def add(self, edge):
        """
        Add an edge to our internal representation. not multi-thread safe
        @param edge: edge to add
        @type  edge: Edge
        """

        # see note in __init__
        def update_map(map, key, edge):
            if key in map:
                l = map[key]
                if not edge in l:
                    l.append(edge)
                    return True
                else:
                    return False
            else:
                map[key] = [edge]
                return True

        updated = update_map(self.edges_by_start, edge.key, edge)
        updated = update_map(self.edges_by_end, edge.rkey, edge) or updated
        return updated

    def add_edges(self, start, dest, direction, label=''):
        """
        Create Edge instances for args and add resulting edges to edge
        list. Convenience method to avoid repetitve logging, etc...
        @param edge_list: data structure to add edge to
        @type  edge_list: EdgeList
        @param start: name of start node. If None, warning will be logged and add fails
        @type  start: str
        @param dest: name of start node. If None, warning will be logged and add fails
        @type  dest: str
        @param direction: direction string (i/o/b)
        @type  direction: str
        @return: True if update occured
        @rtype: bool
        """

        # the warnings should generally be temporary, occuring of the
        # master/node information becomes stale while we are still
        # doing an update
        updated = False
        if not start:
            logger.warn("cannot add edge: cannot map start [%s] to a node name", start)
        elif not dest:
            logger.warn("cannot add edge: cannot map dest [%s] to a node name", dest)
        else:
            for args in edge_args(start, dest, direction, label):
                updated = self.add(Edge(*args)) or updated
        return updated

    def delete_all(self, node):
        """
        Delete all edges that start or end at node
        @param node: name of node
        @type  node: str
        """

        def matching(map, pref):
            return [map[k] for k in map.keys() if k.startswith(pref)]

        pref = node + "|"
        edge_lists = matching(self.edges_by_start, pref) + matching(self.edges_by_end, pref)
        for el in edge_lists:
            for e in el:
                self.delete(e)

    def delete(self, edge):
        # see note in __init__
        def update_map(map, key, edge):
            if key in map:
                edges = map[key]
                if edge in edges:
                    edges.remove(edge)
                    return True

        update_map(self.edges_by_start, edge.key, edge)
        update_map(self.edges_by_end, edge.rkey, edge)


class Edge(object):
    """
    Data structure for representing ROS node graph edge
    """

    __slots__ = ['start', 'end', 'label', 'key', 'rkey']

    def __init__(self, start, end, label=''):
        self.start = start
        self.end = end
        self.label = label
        self.key = "%s|%s" % (self.start, self.label)
        # reverse key, indexed from end
        self.rkey = "%s|%s" % (self.end, self.label)

    def __ne__(self, other):
        return self.start != other.start or self.end != other.end

    def __str__(self):
        return "%s->%s" % (self.start, self.end)

    def __eq__(self, other):
        return self.start == other.start and self.end == other.end


def edge_args(start, dest, direction, label):
    """
    compute argument ordering for Edge constructor based on direction flag
    @param direction str: 'i', 'o', or 'b' (in/out/bidir) relative to \a start
    @param start str: name of starting node
    @param start dest: name of destination node
    """
    edge_args = []
    if direction in ['o', 'b']:
        edge_args.append((start, dest, label))
    if direction in ['i', 'b']:
        edge_args.append((dest, start, label))
    return edge_args


class Graph(object):
    """
    Utility class for polling ROS statistics from running ROS graph.
    Not multi-thread-safe
    """

    def __init__(self, ros: Ros, node_ns='/', topic_ns='/'):
        self.master = ros

        self.node_ns = node_ns or '/'
        self.topic_ns = topic_ns or '/'

        # ROS nodes
        self.nn_nodes = set([])
        # ROS topic nodes
        self.nt_nodes = set([])

        # ROS nodes that aren't responding quickly
        self.bad_nodes = {}
        import threading
        self.bad_nodes_lock = threading.Lock()

        # ROS services
        self.srvs = set([])
        # ROS node->node transport connections
        self.nn_edges = EdgeList()
        # ROS node->topic connections
        self.nt_edges = EdgeList()
        # ROS all node->topic connections, including empty
        self.nt_all_edges = EdgeList()

        # node names to URI map
        self.node_uri_map = {}  # { node_name_str : uri_str }
        # reverse map URIs to node names
        self.uri_node_map = {}  # { uri_str : node_name_str }

        # time we last contacted master
        self.last_master_refresh = 0
        self.last_node_refresh = {}

        # time we last communicated with master
        # seconds until master data is considered stale
        self.master_stale = 5.0
        # time we last communicated with node
        # seconds until node data is considered stale
        self.node_stale = 5.0  # seconds

    def set_master_stale(self, stale_secs):
        """
        @param stale_secs: seconds that data is considered fresh
        @type  stale_secs: double
        """
        self.master_stale = stale_secs

    def set_node_stale(self, stale_secs):
        """
        @param stale_secs: seconds that data is considered fresh
        @type  stale_secs: double
        """
        self.node_stale = stale_secs

    def _master_refresh(self):
        """
        @return: True if nodes information was updated
        @rtype: bool
        """
        logger.debug("master refresh: starting")
        updated = False

        nodes = self.master.get_nodes()
        topics = self.master.get_topics()
        services = self.master.get_services()
        node_details = {}
        for node in nodes:
            details = self.master.get_node_details(node)
            node_details[node] = details

        pubs = []
        for topic in topics:
            topic_pubs = []
            for node in node_details.keys():
                if topic in node_details[node]['publishing']:
                    topic_pubs.append(node)
            pubs.append([topic, topic_pubs])

        subs = []
        for topic in topics:
            topic_subs = []
            for node in node_details.keys():
                if topic in node_details[node]['subscribing']:
                    topic_subs.append(node)
            subs.append([topic, topic_subs])

        srvs = []
        for service in services:
            service_pros = []
            for node in node_details.keys():
                if service in node_details[node]['services']:
                    service_pros.append(node)
            srvs.append([service, service_pros])

        nodes = []
        nt_all_edges = self.nt_all_edges
        nt_nodes = self.nt_nodes
        for state, direction in ((pubs, 'o'), (subs, 'i')):
            for topic, l in state:
                if topic.startswith(self.topic_ns):
                    nodes.extend([n for n in l if n.startswith(self.node_ns)])
                    nt_nodes.add(topic_node(topic))
                    for node in l:
                        updated = nt_all_edges.add_edges(
                            node, topic_node(topic), direction) or updated

        nodes = set(nodes)

        srvs = set([s for s, _ in srvs])
        purge = None
        if nodes ^ self.nn_nodes:
            purge = self.nn_nodes - nodes
            self.nn_nodes = nodes
            updated = True
        if srvs ^ self.srvs:
            self.srvs = srvs
            updated = True

        if purge:
            logger.debug("following nodes and related edges will be purged: %s", ','.join(purge))
            for p in purge:
                logger.debug('purging edges for node %s', p)
                self.nn_edges.delete_all(p)
                self.nt_edges.delete_all(p)
                self.nt_all_edges.delete_all(p)

        logger.debug("master refresh: done, updated[%s]", updated)
        return updated

    def _mark_bad_node(self, node, reason):
        try:
            # bad nodes are updated in a separate thread, so lock
            self.bad_nodes_lock.acquire()
            if node in self.bad_nodes:
                self.bad_nodes[node].type = BadNode.DEAD
            else:
                self.bad_nodes[node] = (BadNode(node, BadNode.DEAD, reason))
        finally:
            self.bad_nodes_lock.release()

    def _unmark_bad_node(self, node, reason):
        """
        Promotes bad node to 'wonky' status.
        """
        try:
            # bad nodes are updated in a separate thread, so lock
            self.bad_nodes_lock.acquire()
            bad = self.bad_nodes[node]
            bad.type = BadNode.WONKY
        finally:
            self.bad_nodes_lock.release()

    def update(self):
        self._master_refresh()

