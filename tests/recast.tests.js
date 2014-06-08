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

var recast = require('../lib/recast');

// Check our library is here
exports['recast is present'] = function(test) {
    test.ok(recast, 'recast should be an object');
    test.done();
};

// Check our methods are here
exports['our methods are present'] = function(test) {
    test.ok(recast.set_cellSize);
    test.ok(recast.set_cellHeight);
    test.ok(recast.set_agentHeight);
    test.ok(recast.set_agentRadius);
    test.ok(recast.set_agentMaxClimb);
    test.ok(recast.set_agentMaxSlope);
    test.ok(recast.initWithFileContent);
    test.ok(recast.build);
    test.ok(recast.initCrowd);
    test.ok(recast.initWithFileContent);
    test.ok(recast.findNearestPoint);
    test.ok(recast.findPath);
    test.ok(recast.getRandomPoint);
    test.ok(recast.addCrowdAgent);
    test.ok(recast.updateCrowdAgentParameters);
    test.ok(recast.requestMoveVelocity);
    test.ok(recast.removeCrowdAgent);
    test.ok(recast.crowdRequestMoveTarget);
    test.ok(recast.crowdUpdate);
    test.ok(recast.crowdGetActiveAgents);
    test.done();
};

// Check file loading
exports['can load an .obj file'] = function(test) {
    test.expect(11);

    test.doesNotThrow(function(){

        recast.set_cellSize(1.0);
        recast.set_cellHeight(2.0);
        recast.set_agentHeight(2.0);
        recast.set_agentRadius(0.2);
        recast.set_agentMaxClimb(4.0);
        recast.set_agentMaxSlope(30.0);

        /*
        recast.settings({
            cellSize: 2.0,
            cellHeight: 1.5,
            agentHeight: 2.0,
            agentRadius: 0.2,
            agentMaxClimb: 4.0,
            agentMaxSlope: 30.0
        });
        */

        /**
         * Load an .OBJ file
         */
        recast.OBJUrlLoader('nav_test.obj', function(){

            /**
             * Find a random navigable point on this mesh
             */
            recast.getRandomPoint(recast.cb(function(pt1x, pt1y, pt1z){
                test.ok(typeof pt1x === 'number');
                test.ok(typeof pt1y === 'number');
                test.ok(typeof pt1z === 'number');

                /**
                 * Find the nearest navigable point from 0,0,0 with a maximum extend of 10,10,10
                 */
                recast.findNearestPoint(0, 0, 0, 10, 10, 10, recast.cb(function(pt2x, pt2y, pt2z){
                    test.ok(typeof pt2x === 'number');
                    test.ok(typeof pt2y === 'number');
                    test.ok(typeof pt2z === 'number');


                    /**
                     * Find the nearest navigable polygon from 0,0,0 with a maximum extend of 10,10,10
                     */
                    recast.findNearestPoly(0, 0, 0, 10, 10, 10, recast.cb(function(count, vertice1, vertice2 /* ... */){
                        test.ok(count > 0);

                        var vertices = Array.prototype.slice.call(arguments, 1);
                        test.ok(vertices && typeof vertices.length !== 'undefined');
                        test.ok(vertices && vertices.length > 0);


                        /**
                         * Find the shortest possible path from pt1 to pt2
                         */
                        recast.findPath(pt1x, pt1y, pt1z, pt2x, pt2y, pt2z, 1000, recast.cb(function(path){
                            test.ok(path && typeof path.length !== 'undefined');

                        }));
                    }));
                }));
            }));


            // addCrowdAgent
            // removeCrowdAgent
            // crowdUpdate
            // crowdGetActiveAgents

        });
    });
    test.done();
};
