function varargout = PIDController(varargin)
% PIDCONTROLLER MATLAB code for PIDController.fig
%      PIDCONTROLLER, by itself, creates a new PIDCONTROLLER or raises the existing
%      singleton*.
%
%      H = PIDCONTROLLER returns the handle to a new PIDCONTROLLER or the handle to
%      the existing singleton*.
%
%      PIDCONTROLLER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PIDCONTROLLER.M with the given input arguments.
%
%      PIDCONTROLLER('Property','Value',...) creates a new PIDCONTROLLER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PIDController_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PIDController_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PIDController

% Last Modified by GUIDE v2.5 23-Jul-2017 12:55:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @PIDController_OpeningFcn, ...
    'gui_OutputFcn',  @PIDController_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before PIDController is made visible.
function PIDController_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PIDController (see VARARGIN)

% Choose default command line output for PIDController
handles.output = hObject;

% Update handles structure

guidata(hObject, handles);


% UIWAIT makes PIDController wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PIDController_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

title('Arduino PID Controller','FontSize',25);
xlabel('Elapsed Time (sec)','FontSize',15);
ylabel('Speed','FontSize',15);
grid on;

%% PLOT DATA
last_speed = 0;
t = 1;  %index
y = zeros(1, 10000); %speed values
x = zeros(0,10000); %time values
timer_array = zeros(0,10000); %time values
data_array = zeros(0,10000);
avg_speed_array = zeros(0,100);
data_counter = 1;
correction_counter = 0;

start_timer = 0;
end_timer = 0;
peak = 0;
lowest = 0;
past_y = 0;

avg_speed_counter = 0;

tic % Start timer
while toc <= 100000
    
    y(t) = smooth(fscanf(handles.s, '%f'), 10, 'moving');        
    set(handles.current_speed, 'String', y(t));
    
    percent_diffs = abs(((y(t)-past_y)/past_y)*100);
    diffs = strcat(num2str(percent_diffs),'%');
    set(handles.steady_state, 'String', diffs);
    
    x(t) = toc;
    
    % Now plot the data
    if t> 1
        line(handles.axes1,[x(t-1) x(t)],[y(t-1) y(t)]);
        axis(handles.axes1,[x(t)-15,x(t)+5,0,300]); %Set the axis
        drawnow limitrate;
    end
    
    current_speed = str2double(get( handles.speed_value, 'String' ));
      
    if last_speed ~= current_speed
        
        set(handles.speed1, 'String', last_speed);
        set(handles.speed2, 'String', current_speed);
        start_timer = toc;
        
        while correction_counter < 5
            
            t = t + 1;
            avg_speed_counter = avg_speed_counter + 1;
            y(t) = smooth(fscanf(handles.s, '%f'), 10, 'moving');
            set(handles.current_speed, 'String', y(t));
            avg_speed_array(avg_speed_counter) = y(t);
            
            x(t) = toc;
            
            % Now plot the data
            line(handles.axes1,[x(t-1) x(t)],[y(t-1) y(t)]);
            axis(handles.axes1,[x(t)-15,x(t)+5,0,300]); %Set the axis
            drawnow limitrate;
            
            current_speed = str2double(get( handles.speed_value, 'String' ));
            
            if last_speed < current_speed
                
                if y(t) > past_y
                    data_array(data_counter) = y(t);
                    data_counter = data_counter + 1;
                    timer_array(data_counter) = toc;
                end
                
            elseif last_speed > current_speed
                
                if y(t) > past_y || y(t) < past_y
                    data_array(data_counter) = y(t);
                    data_counter = data_counter + 1;
                    timer_array(data_counter) = toc;
                end

            end
            
            past_y = y(t);  
            
            if y(t) == current_speed
                correction_counter = correction_counter + 1;
            end
           
        end
        
        if last_speed < current_speed
            [peak, idx] = max(data_array);
            riser = timer_array(idx);
            set(handles.peak_value, 'String', peak);
            set(handles.rise_time, 'String', riser - start_timer);
            percent_diff = abs(((peak-current_speed)/current_speed)*100);
        elseif last_speed > current_speed
            [pks, ~] = findpeaks(data_array,x(t),'MinPeakProminence',5);
            [~, locz] = findpeaks(data_array);
            
            rizer = timer_array(locz);
            
            set(handles.peak_value, 'String', max(pks));
            
            set(handles.rise_time, 'String', rizer - start_timer);
            
            percent_diff = abs(((max(pks)-current_speed)/current_speed)*100);
        end

        last_speed = str2double(get( handles.speed_value, 'String' ));
        set(handles.speed_avg, 'String', mean(avg_speed_array));
        
        avg_speed_array = [];
        data_array = [];
        timer_array = [];
        data_counter = 1;
        correction_counter = 0;
        avg_speed_counter = 0;
             
        end_timer = x(t);
        set(handles.settling_time, 'String', end_timer - start_timer);
              
        diff = strcat(num2str(percent_diff),'%');
        set(handles.over_shoot, 'String', diff);
    end
    
    t = t + 1;

    
end

flushoutput(handles.s);
fclose(handles.s);
delete(handles.s);
clear handles.s;
close all;
guidata(hObject, handles);


function p_value_Callback(hObject, eventdata, handles)
% hObject    handle to p_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of p_value as text
%        str2double(get(hObject,'String')) returns contents of p_value as a double


% --- Executes during object creation, after setting all properties.
function p_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to p_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function i_value_Callback(hObject, eventdata, handles)
% hObject    handle to i_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of i_value as text
%        str2double(get(hObject,'String')) returns contents of i_value as a double


% --- Executes during object creation, after setting all properties.
function i_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to i_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d_value_Callback(hObject, eventdata, handles)
% hObject    handle to d_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d_value as text
%        str2double(get(hObject,'String')) returns contents of d_value as a double


% --- Executes during object creation, after setting all properties.
function d_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function speed_value_Callback(hObject, eventdata, handles)
% hObject    handle to speed_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of speed_value as text
%        str2double(get(hObject,'String')) returns contents of speed_value as a double


% --- Executes during object creation, after setting all properties.
function speed_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to speed_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in send_btn.
function send_btn_Callback(hObject, eventdata, handles)
% hObject    handle to send_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
pvalue = get( handles.p_value, 'String' );
ivalue = get( handles.i_value, 'String' );
dvalue = get( handles.d_value, 'String' );
speedvalue = get( handles.speed_value, 'String' );


command = strcat('1',',',speedvalue,',',pvalue,',',ivalue,',',dvalue);
fprintf(handles.s,'%s',command); % Send data to arduino...


guidata(hObject,handles)
% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


try
    delete(instrfindall); %Close all open ports
    
    
    %% INITIALIZE SERIAL PORT
    handles.s = serial('COM5'); %Initialize COM number
    set(handles.s,'BaudRate',19200);  %Set Baud Rate should be equal with Arduino
    set(handles.s,'DataBits', 8); % 8 Data Bits
    set(handles.s,'StopBits', 1); % 1 Stop Bit
    fopen(handles.s);
    
    
    
    guidata(hObject, handles);
    return;
    
catch
    fprintf(1, 'Arduino not found. Check the connection and restart the program. \n');
    return;
end


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

try
    fprintf(handles.s,'%s','0,0,0.0,0.0,0.0'); % SEND DATA TO ARDUINO TO STOP THE MOTOR
    fclose(handles.s);
    delete(handles.s);
catch
    fprintf(1, 'Arduino not found. Check the connection and restart the program. \n');
    return;
end
