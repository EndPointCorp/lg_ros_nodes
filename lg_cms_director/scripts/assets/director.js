angular.module('director', ['ngSanitize'])
 .controller('DirectorController', ['$scope', '$http', function($scope, $http) {
  $scope.tabs = [{
    title: 'FREE FLIGHT',
    url: 'freeflight.tpl.html'
  }, {
    title: 'PRESENTATIONS',
    url: 'presentations.tpl.html'
  }, {
    title: 'PANORAMAS',
    url: 'panoramas.tpl.html'
  }, {
    title: 'SEARCH',
    url: 'search.tpl.html'
  }];
  $scope.currentTab = 'freeflight.tpl.html';
  $scope.onClickTab = function(tab) {
    $scope.currentTab = tab.url;
  }
  $scope.$on('$includeContentLoaded', function() {
    if ($scope.currentTab == 'search.tpl.html') {
      // Load the keyboard
      var searchbox = document.getElementById('searchbox');
      VKI_attach(searchbox);
      searchbox.focus();
      // VKI_show() doesn't seem to work immediately; add a delay
      setTimeout(function() {
        VKI_show(searchbox);
      }, 10);
    }
  });
  $scope.isActiveTab = function(tabUrl) {
    return tabUrl == $scope.currentTab;
  }

  $scope.groups = [];
  $scope.group = {};
  $scope.presentation = {};
  $scope.scene = {};
  $scope.groupSelected = false;
  $scope.presentationSelected = false;

  function DirectorSocket(channel) {
   url = "ws://" + location.host + '/' + channel;
   ws = new WebSocket(url);
   ws.onopen = function() {
    console.log("Connected to " + url);
   };
   ws.onmessage = function(message) {
     //console.log("Received data from websocket: " + message.data);
     $scope[channel] = (JSON.parse(message.data));
     $scope.$apply(); // crucial
   };
   return ws;
  };

  function responseHandler(ws) {
   handler = function(data, status, headers, config) {
    data = JSON.stringify(data)
    //console.log(data);
    ws.send(data); };
   return handler;
  }

  var wsScene = new DirectorSocket('scene');
  var wsGroup = new DirectorSocket('group');
  var wsPresentation = new DirectorSocket('presentation');

  $scope.fetch_groups = function() {
   $http({method: 'GET', url: '/director_api/presentationgroup/'}).success(
    function(data, status, headers, config){
     $scope.groups = data.objects;
     $scope.apply; // may not be needed?
    }
   )
  };

  //TODO Refactor these three functions together.
  $scope.fetch_group = function(resource_uri) {
   console.log("Fetching Group " + resource_uri);
   $http({method: 'GET', url: resource_uri}).success(responseHandler(wsGroup));
  };
  $scope.$watch('group', function(selected) {
   if (Object.getOwnPropertyNames(selected).length) {
    console.log(selected);
    $scope.groupSelected = true;
   }
  });

  $scope.presentation_back = function() {
   $scope.groupSelected = false;
  }
  $scope.fetch_presentation = function(resource_uri) {
   console.log("Fetching Presentation " + resource_uri);
   $http({method: 'GET', url: resource_uri}).success(
    responseHandler(wsPresentation));
  };
  $scope.$watch('presentation', function(selected) {
   if (Object.getOwnPropertyNames(selected).length) {
    console.log(selected);
    $scope.presentationSelected = true;
   }
  });

  $scope.scene_back = function() {
   $scope.presentationSelected = false;
  }
  $scope.fetch_scene = function(resource_uri) {
   console.log("Loading Scene " + resource_uri);
   $http({method: 'GET', url: resource_uri}).success(responseHandler(wsScene));
  };

  $scope.fetch_groups();
 }]);
