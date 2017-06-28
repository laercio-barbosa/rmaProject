% Author: Laercio Barbosa 
% Date: 31/05/2017
% SLAM frontend for V-REP .
%
% Based on: mapMakerGUI from Jai Juneja (See Copyright info bellow)
%    - http://www.jaijuneja.com/blog/2013/05/simultaneous-localisation-mapping-matlab/
%    - https://github.com/jaijuneja/ekf-slam-matlab
%
% COPYRIGHT INFO FROM mapMakerGUI
% ===============================
% 
% This software was written and developed by Jai Juneja as part of an 
% undergraduate project in the Department of Engineering Science, University of 
% Oxford. If you have any questions, I can be contacted at: 
% jai.juneja@balliol.ox.ac.uk or jai.juneja@gmail.com
% 
% Please feel free to use, modify and distribute the software for personal or 
% research purposes, along with acknowledgement of the author and inclusion of
% this copyright information.
% 
% Some of the code included has been adapted from other software. They are 
% acknowledged as follows:
% 
% Code for Jacobian transformations was adapted from a SLAM course by Joan Sola 
% (http://www.joansola.eu/JoanSola/eng/course.html)
% ICP algorithm in doICP.m was adapted from code written by Ajmal Saeed Mian 
% (relevant copyright informtion included in the file).
% Any 3rd party code that has been untouched is in the folder 3rd-party

function varargout = slamViewerGUI(varargin)
% slamViewerGUI MATLAB code for slamViewerGUI.fig
%      slamViewerGUI, by itself, creates a new slamViewerGUI or raises the existing
%      singleton*.
%
%      H = slamViewerGUI returns the handle to a new slamViewerGUI or the handle to
%      the existing singleton*.
%
%      slamViewerGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in slamViewerGUI.M with the given input arguments.
%
%      slamViewerGUI('Property','Value',...) creates a new slamViewerGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before slamViewerGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to slamViewerGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help slamViewerGUI

% Last Modified by GUIDE v2.5 17-Jun-2017 14:22:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @slamViewerGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @slamViewerGUI_OutputFcn, ...
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

% --- Executes just before slamViewerGUI is made visible.
function slamViewerGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to slamViewerGUI (see VARARGIN)

% Choose default command line output for slamViewerGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

addpath('3rd-party', 'tools')

global Lmks LmkGraphics  DefaultText AxisDim RunTime World

World = [];

Lmks = [];
LmkGraphics = line(...
    'parent',handles.mainAxes, ...
    'linestyle','none', ...
    'marker','+', ...
    'color','b', ...
    'xdata',[], ...
    'ydata',[]);


DefaultText = 'Select a command...';

AxisDim = 5;
axes(handles.mainAxes)
axis([-AxisDim AxisDim -AxisDim AxisDim])
axis square
grid minor
tick_div = (AxisDim*2)/10; % Set 10 minor grid divison lines
set(gca,'xtick',(-AxisDim:tick_div:AxisDim))
set(gca,'ytick',(-AxisDim:tick_div:AxisDim))
set(handles.AxisDimVar, 'String', AxisDim)

RunTime = 2000;
set(handles.RunTimeVar, 'String', RunTime)
set(handles.map1, 'XTick', [], 'YTick', [])


% --- Outputs from this function are returned to the command line.
function varargout = slamViewerGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in AddLmk.
function AddLmk_Callback(hObject, eventdata, handles)
% hObject    handle to AddLmk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Lmks AxisDim DefaultText
clicked = 0;
while ~clicked
    [x,y]=ginputax(handles.mainAxes,1);
    if abs(x) < AxisDim && abs(y) < AxisDim
        Lmks = [Lmks [x;y]];
        axes(handles.mainAxes);
        %hold on
        plotItems(Lmks, 'landmarks');
        clicked = 1;
    end
end

% --- Executes on button press in DelLmk.
function DelLmk_Callback(hObject, eventdata, handles)
% hObject    handle to DelLmk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Lmks AxisDim DefaultText
clicked = 0;
while ~clicked
    if ~isempty(Lmks)
        p = ginputax(handles.mainAxes,1);
        if abs(p(1))<AxisDim && abs(p(2))<AxisDim
            i = nearestNeighbour(Lmks, p);
            % Remove nearest neighbour from Lmks
            lmk_deleted = Lmks(:,i);
            Lmks(:,i) = [];

            axes(handles.mainAxes);
            hold on
            plotItems(Lmks, 'landmarks');
            clicked = 1;
        end
    else
        clicked = 1;
    end
end

% --- Executes on button press in doSLAM.
function doSLAM_Callback(hObject, eventdata, handles)
% hObject    handle to doSLAM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global AxisDim World Map1 isSlamRunning
if (isequal(get(handles.doSLAM,  'String'), 'Run SLAM'))
    set(handles.helpBox, 'String', 'Connecting to V-REP...');
    set(handles.doSLAM,  'String', 'Stop SLAM');
    World = ekfSLAM(handles, AxisDim);
    set(handles.doSLAM,  'String', 'Run SLAM');
    if (~isempty(World))
        Map1 = World.gridmap_greyscale;
    end
else
    isSlamRunning = false;
    set(handles.doSLAM,  'String', 'Run SLAM');
    set(handles.helpBox, 'String', 'Stopping SLAM...');
end

function plotItems(Items, ItemType)
global LmkGraphics
if strcmp(ItemType, 'landmarks')
    set(LmkGraphics, 'xdata', Items(1, :), 'ydata', Items(2, :))
end

function i = nearestNeighbour(Items, p)
diff2 = (Items(1,:)-p(1)).^2 + (Items(2,:)-p(2)).^2;
i= find(diff2 == min(diff2));
i= i(1);


% --- Executes on button press in clearMap.
function clearMap_Callback(hObject, eventdata, handles)
% hObject    handle to clearMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Clear axes:
global DefaultText
cla;
% Re-initialise map variables:
slamViewerGUI_OpeningFcn(hObject, eventdata, handles);
set(handles.helpBox, 'String', ['Map cleared! ' DefaultText])

function AxisDimVar_Callback(hObject, eventdata, handles)
% hObject    handle to AxisDimVar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AxisDimVar as text
%        str2double(get(hObject,'String')) returns contents of AxisDimVar as a double


% --- Executes during object creation, after setting all properties.
function AxisDimVar_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AxisDimVar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in setAxes.
function setAxes_Callback(hObject, eventdata, handles)
% hObject    handle to setAxes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global AxisDim DefaultText
AxisDim = str2double(get(handles.AxisDimVar, 'String'));
axes(handles.mainAxes)
axis([-AxisDim AxisDim -AxisDim AxisDim])
grid minor
tick_div = (AxisDim*2)/10; % Set 10 minor grid divison lines
set(gca,'xtick',(-AxisDim:tick_div:AxisDim))
set(gca,'ytick',(-AxisDim:tick_div:AxisDim))
set(handles.helpBox, 'String', ['Axis size set to: ' num2str(AxisDim) '. ' DefaultText])


function RunTimeVar_Callback(hObject, eventdata, handles)
% hObject    handle to RunTimeVar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of RunTimeVar as text
%        str2double(get(hObject,'String')) returns contents of RunTimeVar as a double


% --- Executes during object creation, after setting all properties.
function RunTimeVar_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RunTimeVar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in setRunTime.
function setRunTime_Callback(hObject, eventdata, handles)
% hObject    handle to setRunTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global RunTime DefaultText
RunTime = str2double(get(handles.RunTimeVar, 'String'));
set(handles.helpBox, 'String', ['Run time set to: ' num2str(RunTime) '. ' DefaultText])


% --- Executes on button press in saveGridMap.
function saveGridMap_Callback(hObject, eventdata, handles)
% hObject    handle to saveGridMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global DefaultText Map1 World

if ~isempty(World)
    mapFormat = get(handles.saveGridMapType, 'Value');
    gridmap = Map1;  % 8-bit map
    
    switch mapFormat
        case 1 % Image
            gridmap = imresize(flipud(gridmap), round(World.map_res * 20), 'nearest');
            imwrite(gridmap, 'gridmap.tiff');
            % add screen size and comment
            set(handles.helpBox, 'String', ['Map saved as gridmap.tiff! ' DefaultText])
        case 2 % .mat file
            map_res = World.map_res;
            seed = {'*.mat','MAT-files (*.mat)'};
            [fn,pn] = uiputfile(seed, 'Save Occupancy Grid Map');
            if fn==0, return, end

            fnpn = strrep(fullfile(pn,fn), '''', '''''');
            save(fnpn, 'gridmap', 'map_res');

            set(handles.helpBox, 'String', ['Grid map saved! ' DefaultText])     
        case 3 % .txt file
            map_res = World.map_res;
            seed = {'*.txt','Text-files (*.txt)'};
            [fn,pn] = uiputfile(seed, 'Save Occupancy Grid Map');
            if fn==0, return, end

            fnpn = strrep(fullfile(pn,fn), '''', '''''');
            save(fnpn, 'gridmap', 'map_res', '-ascii');

            set(handles.helpBox, 'String', ['Grid map saved! ' DefaultText])     
            
    end
else
    errordlg(['The map you are trying to save is currently empty.' ...
        'First run a simulation to generate a map.'],'Map not saved!')
end
        

% --- Executes on selection change in saveGridMapType.
function saveGridMapType_Callback(hObject, eventdata, handles)
% hObject    handle to saveGridMapType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns saveGridMapType contents as cell array
%        contents{get(hObject,'Value')} returns selected item from saveGridMapType


% --- Executes during object creation, after setting all properties.
function saveGridMapType_CreateFcn(hObject, eventdata, handles)
% hObject    handle to saveGridMapType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
