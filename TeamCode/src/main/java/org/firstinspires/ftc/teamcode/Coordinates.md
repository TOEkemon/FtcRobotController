--- FOLDER: Boundaries ---
y = -72 { -72 < x < 72 }
y = 72 { -72 < x < 72 }
x = -72 { -72 < y < 72 }
x = 72 { -72 < y < 72 }

--- FOLDER: Loading Zone Red ---
polygon((-72, -48), (-48, -48), (-48, -72), (-72, -72))

--- FOLDER: Left Upper Launch Line ---
y = -x { -58.20359 < x < 0 }

--- FOLDER: Right Upper Launch Line ---
y = x { 0 < x < 58.20359 }

--- FOLDER: Left Lower Launch Line ---
y = x - 48 { -24 < x < 0 }

--- FOLDER: Right Lower Launch Line ---
y = -x - 48 { 0 < x < 24 }

--- FOLDER: Gate Zone Red ---
y = 1 { 48 < x < 58 }
y = -1 { 48 < x < 58 }

--- FOLDER: Gate Zone Blue ---
y = 1 { -58 < x < -48 }
y = -1 { -58 < x < -48 }

--- FOLDER: Loading Zone Blue ---
polygon((72, -48), (48, -48), (48, -72), (72, -72))

--- FOLDER: Grid Vertical ---
x = -48 { -72 < y < 72 }
x = -24 { -72 < y < 72 }
x = 0 { -72 < y < 72 }
x = 24 { -72 < y < 72 }
x = 48 { -72 < y < 72 }

--- FOLDER: Grid Horizontal ---
y = -48 { -72 < x < 72 }
y = -24 { -72 < x < 72 }
y = 0 { -72 < x < 72 }
y = 24 { -72 < x < 72 }
y = 48 { -72 < x < 72 }

--- FOLDER: Red Side Secret Tunnel ---
polygon((72, 72), (65.75, 72), (65.75, -48), (72, -48))

--- FOLDER: Blue Side Secret Tunnel ---
polygon((-72, 72), (-65.75, 72), (-65.75, -48), (-72, -48))

--- FOLDER: Blue Alliance Area ---
polygon((126, 24), (72, 24), (72, -72), (126, -72))

--- FOLDER: Depot Line Blue ---
y = (96/71) x + 9720/71 { -65.75 < x < -48 }

--- FOLDER: Depot Line Red ---
y = -(96/71) x + 9720/71 { 48 < x < 65.75 }

--- FOLDER: Red Alliance Area ---
polygon((-126, 24), (-72, 24), (-72, -72), (-126, -72))

--- FOLDER: Blue Side Spike Lines ---
y = -36 { -53 < x < -43 }
y = -12 { -53 < x < -43 }
y = 12 { -53 < x < -43 }

--- FOLDER: Red Side Spike Lines ---
y = -36 { 43 < x < 53 }
y = -12 { 43 < x < 53 }
y = 12 { 43 < x < 53 }

--- FOLDER: Origin ---
(0, 0)

--- FOLDER: Red Base Zone ---
polygon((-43, -47), (-43, -29), (-25, -29), (-25, -47))

--- FOLDER: Blue Base Zone ---
polygon((25, -47), (25, -29), (43, -29), (43, -47))

--- FOLDER: Red Goal Post ---
polygon((48, 72), (65.75, 48), (65.75, 72))

--- FOLDER: Blue Goal Post ---
polygon((-48, 72), (-65.75, 48), (-65.75, 72))

--- FOLDER: April Tag Obelisk ---
polygon((-10, 72), (0, 82), (10, 72))

--- FOLDER: Launch Zone Near Goal ---
polygon((0, 0), (-58, 58), (-48, 72), (48, 72), (58, 58))

--- FOLDER: Launch Zone Near Loading ---
polygon((-24, -72), (0, -48), (24, -72))

--- FOLDER: Red Field Side ---
polygon((-72, 72), (0, 72), (0, -72), (-72, -72))

--- FOLDER: Blue Field Side ---
polygon((72, 72), (0, 72), (0, -72), (72, -72))