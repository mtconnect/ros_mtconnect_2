App.mt_connect_updates = App.cable.subscriptions.create "MtConnectUpdatesChannel",
  connected: ->
    # Called when the subscription is ready for use on the server

  disconnected: ->
    # Called when the subscription has been terminated by the server

  received: (data) ->
    # Called when there's incoming data on the websocket for this channel
    $("#" + data['id']).html(data['value'])
