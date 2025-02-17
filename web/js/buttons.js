
// helios_settings();

function helios_settings_dialog(){

	var id = 'helios_settings';
	if($('#'+id).length){$('#'+id).dialog('destroy').remove();}
	$('<div style="padding:20px;"></div>').attr('id', id).appendTo( 'body' );

    $.ajax({url: '/currentSettings', method: "GET", dataType:'json', success: function(data){helios_settings(data);}});

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

}
	
function helios_settings(data) {
	
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
		'integration_time_tof_1':{'table':'limu_settings','name':'Integration Time ToF 1','units':'us','range':[0,4000],'step':1},
		'integration_time_tof_2':{'table':'limu_settings','name':'Integration Time ToF 2','units':'us','range':[0,4000],'step':1},
		'integration_time_tof_3':{'table':'limu_settings','name':'Integration Time ToF 3','units':'us','range':[0,4000],'step':1},
		'min_amplitude':{'table':'limu_settings','name':'Min Amplitude','units':null,'range':[0,100000],'step':1},
		'roi_left_x':{'table':'limu_settings','name':'Region of Interest Left X','units':'pts','range':[0,319],'step':1},
		'roi_right_x':{'table':'limu_settings','name':'Region of Interest Right X','units':'pts','range':[0,319],'step':1},
		'roi_top_y':{'table':'limu_settings','name':'Region of Interest Top Y','units':'pts','range':[0,239],'step':1},
		'roi_bottom_y':{'table':'limu_settings','name':'Region of Interest Bottom Y','units':'pts','range':[0,239],'step':1},
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
						<input type="number" step=50 min="0" max="4000" class="form-control" name="'+slider+'" id="'+slider+'">\
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
			$.ajax({ type: 'POST', url: '/applySettings', data: json, contentType: "application/json; charset=utf-8", success: function(data){
				data = JSON.parse(data);
				helios_settings(data);
			}});
		},250);		
	});
	$('#'+id+' .display_settings .form-control').off().on('change', function() {
		let name = $(this).attr('name');
		let val = parseInt($(this).val());
		let json = {}; json[name] = val; json = JSON.stringify(json);
		clearTimeout(post_timeout);
		post_timeout = setTimeout(function(){
			$.ajax({ type: 'POST', url: '/displaySettings', data: json, contentType: "application/json; charset=utf-8", success: function(data){
				data = JSON.parse(data);
				helios_settings(data);
			}});
		},250);
	});

	let size = 0.01;
	let object = scene.getObjectByName('pointcloud');
	if(typeof object != 'undefined'){size = object.material.size;}
	$( "#point_size_range" ).slider( "option", "values",  [size] );
	$("#point_size").val(size).off().on('change', function(){
		let size = $("#point_size").val();
		let object = scene.getObjectByName('pointcloud');
		object.material.size = size;
	});
	$("#frame_rate").val(Math.round(1000*1/frame_time)).off().on('change', function(){
		window.frame_time = 1000*1/$("#frame_rate").val();
		console.log(frame_time);
	});

}

$('.helios_settings').on('click', helios_settings_dialog);