import * as THREE from "https://esm.sh/three";
import { OrbitControls } from "https://esm.sh/three/examples/jsm/controls/OrbitControls.js";
import {CSS2DRenderer,CSS2DObject} from 'https://esm.sh/three/addons/renderers/CSS2DRenderer.js';

initialize_canvas();

let m_pressed = false;
let pause = false;
let frame_time = 111;

function buttons(tjs){

    $('.helios_settings').on('click', function(){
        helios_settings(tjs);
    });
    
    $(document).on('keydown', null, 'ctrl+space', function(){
        if(!pause){ pause = true; }
        else{ pause = false; update_scene(tjs); }
    });
    
    $(document).on('keydown', null, 'm', function(){m_pressed = true;});
    $(document).on('keyup', null, 'm', function(){m_pressed = false;});

}

function initialize_canvas() {
    
	let tjs = {'renderer':null,'labelRenderer':null,'scene':null,'camera':null,'mouse':null};

    let canvas = '.viewer';
	const innerWidth = $(canvas).innerWidth();
	const innerHeight = $(canvas).innerHeight();
	
	tjs.renderer = new THREE.WebGLRenderer({antialias: true});
	// renderer.setPixelRatio(window.devicePixelRatio);
	tjs.renderer.setSize( innerWidth, innerHeight );
	tjs.renderer.setClearColor(new THREE.Color(0xffffff), 0);
	$(canvas).html(tjs.renderer.domElement);

    tjs.scene = new THREE.Scene();

    tjs.scene.background = new THREE.Color(0xffffff);

	let perspective_camera = new THREE.PerspectiveCamera(30, innerWidth/innerHeight, 0.01, 10000);
	perspective_camera.position.set(2,2,2);
	// perspective_camera.up.set(0,0,1);
	tjs.perspective_camera = perspective_camera;

	let orthographic_camera = new THREE.OrthographicCamera(innerWidth/-2, innerWidth/2, innerHeight/2, innerHeight/-2, 0.01, 10000);
	orthographic_camera.position.set(0,0,1000);
	orthographic_camera.up.set(0,0,1);
	tjs.orthographic_camera = orthographic_camera;

	tjs.camera = tjs.perspective_camera;
	tjs.scene.add(tjs.camera);

	// Lights
	let ambientLight = new THREE.AmbientLight( 0x333333 );
	tjs.scene.add(ambientLight);
	let light = new THREE.DirectionalLight( 0xFFFFFF, 0.9 );
	light.position.set( 1,1,1 );
	tjs.scene.add(light);

	// Controls
	tjs.orbit_controls = new OrbitControls(tjs.camera, tjs.renderer.domElement);
	tjs.orbit_controls.minDistance = 0.5;
	tjs.orbit_controls.maxDistance = 10000;

	// Setup labels
	tjs.labelRenderer = new CSS2DRenderer();
	tjs.labelRenderer.setSize(innerWidth, innerHeight);
	tjs.labelRenderer.domElement.style.position = 'absolute';
	tjs.labelRenderer.domElement.style.top = '0px';
	tjs.labelRenderer.domElement.style.pointerEvents = 'none';
	$(canvas).append(tjs.labelRenderer.domElement);
	
	const labelDiv = document.createElement('div');
	labelDiv.className = 'threejslabel';
	const label = new CSS2DObject(labelDiv);
	label.visible = false;
	tjs.scene.add(label);

    // Setup mouse events and raycaster
	tjs.mouse = new THREE.Vector2();
    let measure = [null,null];
	let mousedown = false;
	let raycaster = new THREE.Raycaster();
    raycaster.params.Points.threshold = 0.1;
	$(canvas).on('mousedown', function(e){mousedown = true;});
	$(canvas).on('mouseup', function(e){mousedown = false;});
	$(canvas).on('mousemove', function(e){
		let clientX = e.clientX;
		let clientY = e.clientY;
		tjs.mouse.x = ((clientX-$(canvas)[0].getBoundingClientRect().left)/innerWidth)*2-1;
		tjs.mouse.y = -((clientY-$(canvas)[0].getBoundingClientRect().top)/innerHeight)*2+1;
	});

    // Animation loop
	tjs.renderer.setAnimationLoop(() => {
		tjs.orbit_controls.update();
        raycaster.setFromCamera(tjs.mouse, tjs.camera);
        let intersects = raycaster.intersectObjects(tjs.scene.children, true);
        if(intersects.length>0){
            let mid = Math.floor(intersects.length/10);
            if(intersects.length==2){mid=0;}
            if(typeof(intersects[mid])=='undefined'){return true;}
            let point = intersects[mid].point;
            let index = intersects[mid].index;
            let object = intersects[mid].object;
            if(mousedown && m_pressed){
                if(measure[0]==null){
                    measure[0] = [point.x,point.y,point.z];
                    add_dot(point,tjs);
                }
                else if(measure[1]==null){
                    measure[1] = [point.x,point.y,point.z];
                    add_dot(point,tjs);
                    let x1 = measure[0][0], y1 = measure[0][1], z1 = measure[0][2];
                    let x2 = measure[1][0], y2 = measure[1][1], z2 = measure[1][2];
                    let points = [new THREE.Vector3(x1,y1,z1),new THREE.Vector3(x2,y2,z2)];
                    add_line(points,tjs);
                    const distance3d = Math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
                    const d3dm = Math.round(100*distance3d)/100;
                    const d3dft = Math.floor(distance3d*3.28084);
                    const d3din = parseFloat(distance3d*3.28084%1*12).toFixed(0);
                    let distanceLabelDiv = document.createElement('div');
                    distanceLabelDiv.className = 'threejsdistancelabel';
                    distanceLabelDiv.innerHTML = d3dm+"m<br>"+d3dft+"' "+d3din+'"';
                    let distanceLabel = new CSS2DObject(distanceLabelDiv);
                    distanceLabel.position.set((x1+x2)/2,(y1+y2)/2,(z1+z2)/2);
                    distanceLabel.name = 'measurement';
                    tjs.scene.add(distanceLabel);
                }
                else{
                    let remove_objects = [];
                    measure = [null,null];
                    tjs.scene.traverse(function(object){if(object.name.indexOf('measurement')!=-1){remove_objects.push(object);}});
                    $.each(remove_objects, function(i,object){tjs.scene.remove(object);});
                }
                mousedown = false;
            }
        }
		canvas_render(tjs)
    });

    function add_dot(point,tjs){        
        let x1 = point.x, y1 = point.y, z1 = point.z;
        var dotGeometry = new THREE.BufferGeometry();
        dotGeometry.setAttribute('position', new THREE.Float32BufferAttribute([x1,y1,z1],3));
        var dotMaterial = new THREE.PointsMaterial({size:0.2, color:0x0000ff});
        var dot = new THREE.Points( dotGeometry, dotMaterial );
        dot.name = 'measurement';
        tjs.scene.add( dot );
    }

    function add_line(points,tjs){
        let geometry = new THREE.BufferGeometry().setFromPoints( points );
        let material = new THREE.LineBasicMaterial( { color: 0x0000ff } );
        let line = new THREE.Line( geometry, material );
        line.name = 'measurement';
        tjs.scene.add(line);
    }

    update_scene(tjs)
    buttons(tjs)
}

function canvas_render(tjs) {
	let renderer = tjs.renderer;
	let labelRenderer = tjs.labelRenderer;
	let scene = tjs.scene;
	let camera = tjs.camera;
	renderer.render( scene, camera );
	labelRenderer.render(scene, camera);
}

function update_scene(tjs){

    async function success(data) {

        if(data==null){
            console.log(data);
            await new Promise(r => setTimeout(r, 2000));
            update_scene();
            return true;
        }

        try {
            let points = [];
            let colors = [];
    
            let xyz_size = 12; // in bytes
            let color_size = 3; // in bytes
            let point_size = xyz_size + color_size; // in bytes
            let num_points = data.byteLength / point_size
    
            let pointsView = new DataView(data, 0, num_points * xyz_size)
            let colorsView = new DataView(data, num_points * xyz_size)

            for (let i = 0; i < num_points; i += 1) {
                let x = pointsView.getFloat32((i * xyz_size) + 0, true);
                let y = pointsView.getFloat32((i * xyz_size) + 4, true);
                let z = pointsView.getFloat32((i * xyz_size) + 8, true);
                points.push(x, y, z);
            }

            for (let i = 0; i < num_points; i += 1) {
                let r = colorsView.getUint8((i * color_size) + 0, true) / 255;
                let g = colorsView.getUint8((i * color_size) + 1, true) / 255;
                let b = colorsView.getUint8((i * color_size) + 2, true) / 255;
                colors.push(r, g, b);
            }

            let old_pointcloud = tjs.scene.getObjectByName('pointcloud');
            let size = 0.01;
            if(old_pointcloud){size = old_pointcloud.material.size;}

            let geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(points), 3));
            geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));
            const material = new THREE.PointsMaterial({ size: size, vertexColors: true });
            let pointcloud = new THREE.Points(geometry, material);
            pointcloud.name = 'pointcloud';
            tjs.scene.remove(old_pointcloud);
            tjs.scene.add(pointcloud);

            await new Promise(r => setTimeout(r, frame_time));
            if(!pause){update_scene(tjs);}

        } 
        catch (error) {
            console.log(error)
            await new Promise(r => setTimeout(r, 2000));
            update_scene(tjs);
        }
        
    }
    
    async function error(error){
        console.log(error)
        await new Promise(r => setTimeout(r, 2000));
        update_scene(tjs);
    }
    
    $.ajax({url: 'http://helios.local/asbytes', method: "GET", xhrFields: {responseType: "arraybuffer"}, success, error});
}

function helios_settings(tjs){

	var id = 'helios_settings';
	if($('#'+id).length){$('#'+id).dialog('destroy').remove();}
	$('<div style="padding:20px;"></div>').attr('id', id).appendTo( 'body' );

    $.ajax({url: 'http://helios.local/currentSettings', method: "GET", dataType:'json', success: function(data){
        helios_settings_content(data,tjs);
    }});

	var width = 800; var height = 600; var show = null; var hide = null;
	var position = [($(window).width()-width)/2,($(window).height()-height)/2]; 
	
	$('#'+id).dialog({
		title: "Settings",
		width: width,
		height: height,
		position: {my: 'left top', at: 'left+'+position[0]+' top+'+position[1]},
		show: show, 
		hide: hide,
		buttons: {
			"Close": function() {
				$('#'+id).dialog('close'); 
				setTimeout(function(){$('#'+id).dialog('destroy').remove();},500);
			}
		},
		open: function (event, ui) {
			$(this).closest(".ui-dialog")
			.find(".ui-dialog-titlebar-close")
			.removeClass("ui-dialog-titlebar-close")
			.html("<span class='ui-button-icon-primary ui-icon ui-icon-closethick'></span>")
			.attr("class","ui-button ui-corner-all ui-widget ui-button-icon-only ui-dialog-titlebar-close");
		}
	})
    .css({overflow:"auto"})
	$('#'+id).dialog("open");
	
    function helios_settings_content(data,tjs) {
        
        var id = 'helios_settings';
    
        let channel_options = '';
        for(let i=0; i<=15; i++){
            channel_options += '<option value="'+i+'">'+i+'</option>';
        }
    
        $('#'+id).html(
            '<div>\
                <table class="table table-borderless limu_settings mb-0" style="font-size:0.9em;">\
                    <tr>\
                        <td valign="center" width=180px align="right">Lens Type</td>\
                        <td>\
                            <div class="input-group">\
                                <select class="form-control" name="lens_type">\
                                    <option value="0">Wide Field</option>\
                                    <option value="1">Standard Field</option>\
                                    <option value="2">Narrow Field</option>\
                                </select>\
                            </div>\
                        </td>\
                        <td> </td>\
                        <td width=120px align="right">Frequency</td>\
                        <td>\
                            <div class="input-group">\
                                <select class="form-control" name="frequency_modulation">\
                                    <option value="5">0.75</option>\
                                    <option value="4">1.5</option>\
                                    <option value="3">3</option>\
                                    <option value="2">6</option>\
                                    <option value="1">12</option>\
                                    <option value="0">24</option>\
                                </select>\
                                <span class="input-group-append"><span class="input-group-text">MHz</span></span>\
                            </div>\
                        </td>\
                    </tr>\
                    <tr>\
                        <td valign="center" width=180px align="right">Channel</td>\
                        <td>\
                            <div class="input-group">\
                                <select class="form-control" name="channel">\
                                    '+channel_options+'\
                                </select>\
                            </div>\
                        </td>\
                        <td> </td>\
                        <td width=120px align="right">Image Type</td>\
                        <td>\
                            <div class="input-group">\
                                <select class="form-control" name="image_type">\
                                    <option value="0">Off</option>\
                                    <option value="1">Distance</option>\
                                    <option value="2">Distance & Amplitude</option>\
                                </select>\
                            </div>\
                        </td>\
                    </tr>\
                    <tr>\
                        <td valign="center" width=180px align="right">HDR Mode</td>\
                        <td>\
                            <div class="input-group">\
                                <select class="form-control" name="hdr_mode">\
                                    <option value="0">HDR off</option>\
                                    <option value="1">HDR spatial</option>\
                                    <option value="2">HDR temporal</option>\
                                </select>\
                            </div>\
                        </td>\
                        <td> </td>\
                        <td> </td>\
                        <td> </td>\
                    </tr>\
                    <tr><td colspan=5><hr></td></tr>\
                </table>\
                <table class="table table-borderless display_settings" style="font-size:0.9em;">\
                    <tr>\
                        <td valign="center" width=180px align="right">Point Coloring</td>\
                        <td>\
                            <div class="input-group">\
                                <select class="form-control" name="point_color">\
                                    <option value="0">Camera</option>\
                                    <option value="1">Distance</option>\
                                    <option value="2">Amplitude</option>\
                                    <option value="3">X</option>\
                                    <option value="4">Y</option>\
                                    <option value="5">Z</option>\
                                </select>\
                            </div>\
                        </td>\
                        <td> </td>\
                        <td valign="center" width=180px align="right">Frame Rate Target</td>\
                        <td>\
                            <div class="input-group">\
                                <span class="input-group-prepend"><button class="btn btn-secondary btn-minus" type="button">-</button></span>\
                                <input type="number" step=1 min="0" max="4000" class="form-control" name="frame_rate" id="frame_rate">\
                                <span class="input-group-append"><button class="btn btn-secondary btn-plus" type="button">+</button></span>\
                                <span class="input-group-append"><span class="input-group-text">fps</span></span>\
                            </div>\
                        </td>\
                    </tr>\
                </table>\
            </div>'
        );
    
        let sliders = {
            'integration_time_tof_1':{'table':'limu_settings','name':'Integration Time ToF 1','units':'us','range':[0,4000],'step':100},
            'integration_time_tof_2':{'table':'limu_settings','name':'Integration Time ToF 2','units':'us','range':[0,4000],'step':100},
            'integration_time_tof_3':{'table':'limu_settings','name':'Integration Time ToF 3','units':'us','range':[0,4000],'step':100},
            'min_amplitude':{'table':'limu_settings','name':'Min Amplitude','units':null,'range':[0,100000],'step':100},
            'roi_left_x':{'table':'limu_settings','name':'Region of Interest Left X','units':'pts','range':[0,319],'step':10},
            'roi_right_x':{'table':'limu_settings','name':'Region of Interest Right X','units':'pts','range':[0,319],'step':10},
            'roi_top_y':{'table':'limu_settings','name':'Region of Interest Top Y','units':'pts','range':[0,239],'step':10},
            'roi_bottom_y':{'table':'limu_settings','name':'Region of Interest Bottom Y','units':'pts','range':[0,239],'step':10},
            'point_size':{'table':'display_settings','name':'Display Point Size','units':null,'range':[0,0.05],'step':0.0001},
        };
        $.each(sliders, function(slider, config){
            let units = '';
            if(config['units']!=null){units ='<span class="input-group-append"><span class="input-group-text">'+config['units']+'</span></span>';}
            $('#'+id+' .'+config['table']).append(
                '<tr>\
                    <td valign="center" width=180px align="right">'+config['name']+'</td>\
                    <td colspan=3>\
                        <div class="form-group">\
                            <div id="'+slider+'_range" style="margin-top:8px;margin-left:50px;width:250px;"></div>\
                        </div>\
                    </td>\
                    <td>\
                        <div class="input-group">\
                            <span class="input-group-prepend"><button class="btn btn-secondary btn-minus" type="button">-</button></span>\
                            <input type="number" step='+config['step']+' min="'+config['range'][0]+'" max="'+config['range'][1]+'" class="form-control" name="'+slider+'" id="'+slider+'">\
                            <span class="input-group-append"><button class="btn btn-secondary btn-plus" type="button">+</button></span>\
                            '+units+'\
                        </div>\
                    </td>\
                </tr>'
            )
            $("#"+id+" #"+slider+"_range").slider({
                min:config['range'][0], max:config['range'][1], values: [data[slider]], step: config['step'],
                slide: function(event,ui){$("#"+id+" #"+slider).val(ui.values[0]).trigger('change');}
            });
            $("#"+id+" #"+slider)
            .off().on("change",function(){$( "#"+slider+"_range" ).slider( "option", "values",  [$("#"+id+" #"+slider).val()] );})
            .val(data[slider])
            .trigger('change');
        });
    
        $('#'+id+' .limu_settings').append('<tr><td colspan=5><hr></td></tr>');
    
        $('#'+id+' select[name="lens_type"]').val(data['lens_type']);
        $('#'+id+' select[name="frequency_modulation"]').val(data['frequency_modulation']);
        $('#'+id+' select[name="channel"]').val(data['channel']);
        $('#'+id+' select[name="image_type"]').val(data['image_type']);
        $('#'+id+' select[name="hdr_mode"]').val(data['hdr_mode']);
        $('#'+id+' select[name="point_color"]').val(data['point_color']);
    
        $('#'+id+' .table').css({'font-size':'0.8em !important'});
        $('#'+id+' .form-group').css({'margin-bottom':'5px'});
        $('#'+id+' .form-control').css({'border-radius':'0'});
        $('#'+id+' select').css({'max-width':'150px'});
        $('#'+id+' input').css({'max-width':'150px'});
        $('#'+id+' .input-group').css({'padding':'0px 2px'});
        $('#'+id+' .table td').css({'padding':'2px','vertical-align':'middle'});
        $('#'+id+' .btn').css({'font-size':'0.8rem','min-width':'30px'});
        $('#'+id+' .input-group-text').css({'font-size':'0.8rem','padding':'0px 5px'});
        $('#'+id+' .form-control').css({'padding':'2px 5px','height':'25px'});
        $('#'+id+' .btn-secondary').css({'padding':'0px','font-weight':'normal','height':'25px','border-radius':'0','border':'0','background-color':'lightgray','color':'#333'});
        $('#'+id+' .form-label').css({'font-size':'0.8rem','margin-bottom':'0'});
        $('#'+id+' .selectgroup-button').css({'font-size':'0.8rem','padding':'0px 5px'});
        $('#'+id+' .input-group-text').css({'border-radius':'0','line-height':'1.8'});
    
        $('#'+id+' .btn-minus').off().on('click', function() {
            $(this).parent().siblings('input').val(parseInt($(this).parent().siblings('input').val()) - parseInt($(this).parent().siblings('input').attr('step'))).trigger('change');
        });
        
        $('#'+id+' .btn-plus').off().on('click', function() {
            $(this).parent().siblings('input').val(parseInt($(this).parent().siblings('input').val()) + parseInt($(this).parent().siblings('input').attr('step'))).trigger('change');
        });
    
        var post_timeout = null;
        $('#'+id+' .limu_settings .form-control').off().on('change', function() {
            let name = $(this).attr('name');
            let val = parseInt($(this).val());
            let json = {}; json[name] = val; json = JSON.stringify(json);
            clearTimeout(post_timeout);
            post_timeout = setTimeout(function(){
                $.ajax({ type: 'POST', url: 'http://helios.local/applySettings', data: json, contentType: "application/json; charset=utf-8", success: function(data){
                    helios_settings_content(data,tjs);
                }});
            },250);		
        });
        $('#'+id+' .display_settings .form-control').off().on('change', function() {
            let name = $(this).attr('name');
            let val = parseInt($(this).val());
            let json = {}; json[name] = val; json = JSON.stringify(json);
            clearTimeout(post_timeout);
            post_timeout = setTimeout(function(){
                $.ajax({ type: 'POST', url: 'http://helios.local/displaySettings', data: json, contentType: "application/json; charset=utf-8", success: function(data){
                    helios_settings_content(data,tjs);
                }});
            },250);
        });
    
        console.log(tjs);
    
        let size = 0.01;
        let object = tjs.scene.getObjectByName('pointcloud');
        if(typeof object != 'undefined'){size = object.material.size;}
        $( "#point_size_range" ).slider( "option", "values",  [size] );
        $("#point_size").val(size).off().on('change', function(){
            let size = $("#point_size").val();
            let object = tjs.scene.getObjectByName('pointcloud');
            object.material.size = size;
        });
        $("#frame_rate").val(Math.round(1000*1/frame_time)).off().on('change', function(){
            window.frame_time = 1000*1/$("#frame_rate").val();
        });
    
    }

}