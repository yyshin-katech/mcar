import cantools

db = cantools.database.load_file("./180518_HAD_v5.dbc")


for _ in db.messages:
    print(_)


cmd = db.get_message_by_name("Cmd")

print("cmd")
for _ in cmd.signals:
    print(_)
