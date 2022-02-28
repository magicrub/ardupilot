$(document).ready(function () {
    $("#btn-on").click(function () { $.ajax({ url: "/write/0", success: function (result) { console.log(result); } }); });
    $("#btn-off").click(function () { $.ajax({ url: "/write/1", success: function (result) { console.log(result); } }); });
    //fetch lates system time every 100ms
    setInterval(function () {
        $.ajax({ url: "/read/time", success: function (result) { console.log(result); $("#system-time").html(result.time); } });
    }, 100);
});
