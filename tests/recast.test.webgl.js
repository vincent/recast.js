/*!
 * recast.js
 * https://github.com/vincent/recast.js
 *
 * Copyright 2014 Vincent Lark
 * Released under the MIT license
 */
/*jshint onevar: false, indent:4 */
/*global exports: true, require: true, THREE: true, Stats: true */
'use strict';

var renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setSize(document.body.clientWidth * 0.6, document.body.clientHeight * 0.8);
renderer.domElement.style.position = 'absolute';
renderer.domElement.style.bottom = 0;
renderer.domElement.style.right = 0;
document.body.appendChild(renderer.domElement);
renderer.setClearColorHex(0xFFFFFF, 1.0);
renderer.clear();

var width = renderer.domElement.width;
var height = renderer.domElement.height;
var camera = new THREE.PerspectiveCamera( 45, width / height, 1, 10000);
camera.position.y = 50;
camera.position.z = 50;

var controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.addEventListener('change', function(){
    render();
});

var buildParam = location.search.match(/build=(\w+)/);
var buildMethod = buildParam.length == 2 ? buildParam[1] : 'buildTiled';

var agentsParam = location.search.match(/agents=(\d+)/);
var MAX_AGENTS = (agentsParam && agentsParam.length == 2 ) ? agentsParam[1] : 10;
var MAX_HOPS = 10;

var agentsObjects = [];
var scene = new THREE.Scene();

var agentGeometry = new THREE.CylinderGeometry(0.2, 0.5, 2);

for (var i = 0; i < MAX_AGENTS; i++) {
    var agent = new THREE.Object3D();
    var agentBody = new THREE.Mesh(
        agentGeometry,
        new THREE.MeshBasicMaterial({
          color: '#FF0000'
        })
    );
    agentBody.position.y = 1;
    agent.add(agentBody);

    agent.arrowHelper = new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 1),
        new THREE.Vector3(0, 0, 0),
        1.5,
        0xffff00);
    agent.add(agent.arrowHelper);

    agentsObjects.push(agent);
    scene.add(agent);
}

var light = new THREE.SpotLight();
light.position.set( 170, 330, -160 );
scene.add(light);

var navigationMesh, sequence;

////////////////////////////////

var terrain, agents = [];
var debugDraw = {};
var recast = require('../lib/recast');

recast.setGLContext(renderer.context);

function render () {
    renderer.render(scene, camera);

    if (debugDraw.NavMesh)              { recast.drawObject('NavMesh');             }
    if (debugDraw.NavMeshPortals)       { recast.drawObject('NavMeshPortals');      }
    if (debugDraw.RegionConnections)    { recast.drawObject('RegionConnections');   }
    if (debugDraw.RawContours)          { recast.drawObject('RawContours');         }
    if (debugDraw.Contours)             { recast.drawObject('Contours');            }
    if (debugDraw.HeightfieldSolid)     { recast.drawObject('HeightfieldSolid');    }
    if (debugDraw.HeightfieldWalkable)  { recast.drawObject('HeightfieldWalkable'); }
}

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
    test.ok(recast.findPath, 'findPath');
    test.ok(recast.getRandomPoint, 'getRandomPoint');

    test.ok(recast.addCrowdAgent, 'addCrowdAgent');
    test.ok(recast.updateCrowdAgentParameters, 'updateCrowdAgentParameters');
    test.ok(recast.requestMoveVelocity, 'requestMoveVelocity');
    test.ok(recast.removeCrowdAgent, 'removeCrowdAgent');
    test.ok(recast.crowdRequestMoveTarget, 'crowdRequestMoveTarget');
    test.ok(recast.crowdUpdate, 'crowdUpdate');
    test.ok(recast.crowdGetActiveAgents, 'crowdGetActiveAgents');
    test.done();
};

// Check file loading
exports['handle an agent'] = function(test) {
    test.expect(11);

    recast.set_cellSize(0.5);
    recast.set_cellHeight(0.5);
    recast.set_agentHeight(2);
    recast.set_agentRadius(0.8);
    recast.set_agentMaxClimb(2.9);
    recast.set_agentMaxSlope(45);

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


    var stats = new Stats();
    stats.setMode(1); // 0: fps, 1: ms
    stats.domElement.style.position = 'absolute';
    stats.domElement.style.right = '0px';
    stats.domElement.style.bottom = '0px';
    document.body.appendChild( stats.domElement );
    stats.setMode(0);
    stats.begin();


    var loader = new THREE.OBJLoader();
    loader.load('nav_test.obj', function(object){
        terrain = object;
        object.traverse(function(child) {
            if (child instanceof THREE.Mesh) {
                child.material.side = THREE.DoubleSide;
                // child.material.shading = THREE.FlatShading;
            }
        } );
        scene.add(object);
    });


    /**
     * Load an .OBJ file
     */
    recast.OBJLoader('nav_test.obj', function(){

        recast[buildMethod]();
        recast.initCrowd(1000, 1.0);

        // recast.debugCreateNavMesh(0);
        // recast.debugCreateNavMeshPortals();
        // recast.debugCreateRegionConnections();
        // recast.debugCreateRawContours();
        // recast.debugCreateContours();
        // recast.debugCreateHeightfieldSolid();
        // recast.debugCreateHeightfieldWalkable();

        /**
         * Get navmesh geometry and draw it
         */
        if (location.search.match(/navigationmesh=1/)) {
            recast.getNavMeshVertices(recast.cb(function (vertices) {

                navigationMesh = new THREE.Object3D();
                // var materials = [ new THREE.MeshNormalMaterial() ];
                var materials = [ new THREE.MeshBasicMaterial({
                    color: 0x000055,
                    shading: THREE.FlatShading,
                    side: THREE.DoubleSide,
                    wireframe: false,
                    transparent: true,
                    opacity: 0.3,
                    overdraw: true
                }) ];

                for (var i = 0; i < vertices.length; i++) {
                    if (!vertices[i+2]) { break; }

                    var geometry = new THREE.ConvexGeometry([
                        new THREE.Vector3(   vertices[i].x,   vertices[i].y,   vertices[i].z ),
                        new THREE.Vector3( vertices[i+1].x, vertices[i+1].y, vertices[i+1].z ),
                        new THREE.Vector3( vertices[i+2].x, vertices[i+2].y, vertices[i+2].z )
                    ]);

                    var child = THREE.SceneUtils.createMultiMaterialObject(geometry, materials);
                    navigationMesh.add(child);

                    i += 2;
                }

                // scene.add(navigationMesh);

                // renderer.render(scene, camera);
            }));
        }

        var polyMesh, polyGrp, polys = {}, polyGrpIndex = 0;
        var paintPolys = function(polygons){
            // poly.faces[].color.setRGB(Math.random(), Math.random(), Math.random());

            if (! polygons) {
                return;
            }

            if (1 || ! polyGrp) {

                polyGrpIndex++;
                if (polyGrpIndex > 200) {
                    polyGrp = new THREE.Object3D();

                    for (var i = 0; i < polygons.length; i++) {

                        var polygon = polygons[i];
                        var key = polygon.vertices[0].x + '-' + polygon.vertices[0].y + '-' + polygon.vertices[0].z + '-' + polygon.vertices[1].x + '-' + polygon.vertices[1].y + '-' + polygon.vertices[1].z + '-' + polygon.vertices[2].x + '-' + polygon.vertices[2].y + '-' + polygon.vertices[2].z;

                        if (polys[key]) {
                            continue;
                        }

                        polyMesh = new THREE.Mesh(
                            new THREE.Geometry(),
                            new THREE.MeshBasicMaterial({
                                color: 0xff0000,
                                shading: THREE.FlatShading,
                                // side: THREE.DoubleSide,
                                wireframe: false,
                                transparent: false,
                                // vertexColors: THREE.FaceColors, // CHANGED
                                overdraw: true
                            })
                        );

                        polys[key] = polyMesh;

                        polyMesh.geometry.vertices.push(new THREE.Vector3( polygon.vertices[0].x, polygon.vertices[0].y, polygon.vertices[0].z ));
                        polyMesh.geometry.vertices.push(new THREE.Vector3( polygon.vertices[1].x, polygon.vertices[1].y, polygon.vertices[1].z ));
                        polyMesh.geometry.vertices.push(new THREE.Vector3( polygon.vertices[2].x, polygon.vertices[2].y, polygon.vertices[2].z ));

                        polyMesh.geometry.faces.push( new THREE.Face3( 0, 1, 2 ) );

                        // Make this poly unwalkable
                        recast.setPolyFlags(
                            polygon.vertices[0].x,
                            polygon.vertices[0].y,
                            polygon.vertices[0].z,
                            2, 2, 2,
                            0
                        );

                        scene.add(polyMesh);
                    }
                }
            }
        };

        window.addObstacle = function(pointX, pointY, pointZ) {
            var radius = 1 + Math.random() * 5;
            var obstacleMesh = new THREE.Mesh(
                new THREE.CylinderGeometry(radius, radius, 2),
                new THREE.MeshBasicMaterial({
                    color: 0xff0000,
                    shading: THREE.FlatShading,
                    side: THREE.DoubleSide,
                    transparent: true,
                    opacity: 0.8,
                    overdraw: true
                })
            );
            obstacleMesh.position.set(pointX, pointY, pointZ);
            scene.add(obstacleMesh);
            recast.addTempObstacle(pointX, pointY, pointZ, radius);
        };

        var circleMesh;
        var paintCircle = function(pointX, pointY, pointZ) {

            if (! circleMesh) {
                circleMesh = new THREE.Mesh(
                    new THREE.CylinderGeometry(0.8, 0.8, 0.2),
                    new THREE.MeshBasicMaterial({
                        color: 0x00ff00,
                        shading: THREE.FlatShading,
                        // side: THREE.DoubleSide,
                        wireframe: false,
                        transparent: false,
                        // vertexColors: THREE.FaceColors, // CHANGED
                        overdraw: true
                    })
                );

                scene.add(circleMesh);
            }

            circleMesh.position.set(pointX, pointY, pointZ);
        };

        recast.vent.on('update', function (agents) {
            for (var i = 0; i < agents.length; i++) {
                var agent = agents[i];

                var angle = Math.atan2(- agent.velocity.z, agent.velocity.x);
                if (Math.abs(agentsObjects[agent.idx].rotation.y - angle) > 0) {
                    agentsObjects[agent.idx].rotation.y = angle;
                }

                agentsObjects[agent.idx].position.set(
                    agent.position.x,
                    agent.position.y,
                    agent.position.z
                );

                // agentsObjects[agent.idx].arrowHelper.setDirection(
                //     agent.velocity.x * 100,
                //     agent.velocity.y * 100,
                //     agent.velocity.z * 100
                // );

                var color = agent.neighbors * 128;
                agentsObjects[agent.idx].children[0].material.color.set(color, color, color);

                if (parseInt(Math.random() * 10) === 5 &&
                    parseInt(Math.random() * 10) === 5 &&
                    parseInt(Math.random() * 10) === 5) {
                    addObstacle(agent.position.x, agent.position.y, agent.position.z, 2);
                }

                if (0 && agent.idx === 0) {

                    // recast.findNearestPoly(
                    //     agentsObjects[agent.idx].position.x,
                    //     agentsObjects[agent.idx].position.y,
                    //     agentsObjects[agent.idx].position.z,
                    //     2, 2, 2,
                    //     recast.cb(paintPoly)
                    // );

                    recast.queryPolygons(
                        agentsObjects[agent.idx].position.x,
                        agentsObjects[agent.idx].position.y,
                        agentsObjects[agent.idx].position.z,
                        0.3, 0.3, 0.3,
                        recast.cb(paintPolys)
                    );

                    // recast.findNearestPoint(
                    //     agentsObjects[agent.idx].position.x,
                    //     agentsObjects[agent.idx].position.y,
                    //     agentsObjects[agent.idx].position.z,
                    //     2, 2, 2,
                    //     recast.cb(paintCircle)
                    // );
                }
            }
        });

        /**
         * Add some agents
         */
        for (var i = 0; i < agentsObjects.length; i++) {
            agents.push(recast.addAgent({
                position: {
                    x: -25.8850,
                    y: -1.64166,
                    z: -5.41350
                },
                radius: 0.8,
                height: 0.5,
                maxAcceleration: 1.0,
                maxSpeed: 2.0,
                updateFlags: 0, // && recast.CROWD_OBSTACLE_AVOIDANCE, // & recast.CROWD_ANTICIPATE_TURNS & recast.CROWD_OPTIMIZE_TOPO & recast.CROWD_SEPARATION,
                separationWeight: 20.0
            }));
        }

        var routes;

        var routes;

        var last = new Date().getTime();
        var animate = function animate (time) {
            window.requestAnimationFrame(animate);

            recast.crowdUpdate(0.1);
            recast.crowdGetActiveAgents();

            last = time;
            render();

            if (stats) stats.update();
        };

        animate(new Date().getTime());

        sequence = function() {
            document.getElementById('sequence').style.display = 'none';
            routes = 0;
            goAway();
        };

        var goAway = function(){
            for (var i = 0; i < agentsObjects.length; i++) {
                (function (i) {
                    recast.getRandomPoint(recast.cb(function(pt2x, pt2y, pt2z){
                        recast.crowdRequestMoveTarget(i, pt2x, pt2y, pt2z);
                        if (++routes < MAX_HOPS) {
                            test.ok(true, 'route ' + routes + ': to ' + Math.round(pt2x, 2) + ',' + Math.round(pt2y, 2)+ ',' + Math.round(pt2z, 2));
                            setTimeout(goAway, 8000 * Math.random());
                        } else {
                            document.getElementById('sequence').style.display = 'block';
                            // test.done();
                        }
                    }));
                })(i);
            }
        };

        sequence();
    });
};
