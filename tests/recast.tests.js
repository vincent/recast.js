/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4 */
/*global exports: true, require: true */
'use strict';

var recast   = require('../lib/recast');

// Check our library is here
exports['recast is present'] = function(test) {
    test.ok(recast, 'recast should be an object');
    test.done();
};

// Check our methods are here
exports['our methods are present'] = function(test) {
    test.ok(recast.set_cellSize, 'set_cellSize');
    test.ok(recast.set_cellHeight, 'set_cellHeight');
    test.ok(recast.set_agentHeight, 'set_agentHeight');
    test.ok(recast.set_agentRadius, 'set_agentRadius');
    test.ok(recast.set_agentMaxClimb, 'set_agentMaxClimb');
    test.ok(recast.set_agentMaxSlope, 'set_agentMaxSlope');

    test.ok(recast.build, 'build');
    test.ok(recast.initCrowd, 'initCrowd');
    test.ok(recast.initWithFileContent, 'initWithFileContent');
    test.ok(recast.findNearestPoint, 'findNearestPoint');
    test.ok(recast.findNearestPoint, 'findNearestPoly');
    test.ok(recast.findPath, 'findPath');
    test.ok(recast.getRandomPoint, 'getRandomPoint');

    test.ok(recast.addCrowdAgent, 'addCrowdAgent');
    test.ok(recast.updateCrowdAgentParameters, 'updateCrowdAgentParameters');
    test.ok(recast.requestMoveVelocity, 'requestMoveVelocity');
    test.ok(recast.removeCrowdAgent, 'removeCrowdAgent');
    test.ok(recast.crowdRequestMoveTarget, 'crowdRequestMoveTarget');
    test.ok(recast.crowdUpdate, 'crowdUpdate');
    test.ok(recast.crowdGetActiveAgents, 'crowdGetActiveAgents');
    test.done('');
};

// Check file loading
exports['load an .obj file'] = function(test) {

    test.expect(5);

    recast.settings({
        cellSize: 0.1,
        cellHeight: 0.05,
        agentHeight: 1.8,
        agentRadius: 0.4,
        agentMaxClimb: 2.0,
        agentMaxSlope: 30.0
    });

    /**
     * Load an .OBJ file
     */
    recast.OBJLoader('nav_test.obj', function(){

        recast.build();

        /**
         * Find a random navigable point on this mesh
         */
        recast.getRandomPoint(recast.cb(function(pt1x, pt1y, pt1z){
            test.ok(typeof pt1x === 'number' && typeof pt1y === 'number' && typeof pt1z === 'number', 'find a random point');


            /**
             * Find the nearest navigable point from 0,0,0 with a maximum extend of 10,10,10
             */
            recast.findNearestPoint(0, 0, 0, 10, 10, 10, recast.cb(function(pt2x, pt2y, pt2z){
                test.ok(typeof pt2x === 'number' && typeof pt2y === 'number' && typeof pt2z === 'number', 'find the nearest point');

                var extend = 10;

                /**
                 * Find the nearest navigable polygon from 0,0,0 with a maximum extend of 10
                 */
                recast.findNearestPoly(0, 0, 0, extend, extend, extend, recast.cb(function(polygon){
                    test.ok(polygon.vertices, 'origin poly has some vertices');

                    test.ok(polygon.vertices && typeof polygon.vertices.length !== 'undefined', 'origin poly has ' + polygon.vertices.length + ' polygon.vertices');

                    /**
                     * Find the shortest possible path from pt1 to pt2
                     */
                    recast.findPath(pt1x, pt1y, pt1z, pt2x, pt2y, pt2z, 1000, recast.cb(function(path){
                        test.ok(path && typeof path.length !== 'undefined', 'found path has ' + path.length + ' segments');

                        test.done();
                    }));
                }));
            }));

        }));
    });
};


