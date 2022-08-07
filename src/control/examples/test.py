import pygame

pygame.init()

screen = pygame.display.set_mode((300, 200))

pressed = pygame.key.get_pressed()

clock = pygame.time.Clock()
is_running = True

while is_running:

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            is_running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                is_running = False

    last_pressed = pressed
    pressed = pygame.key.get_pressed()

    #changed = [idx for idx in range(len(pressed)) if pressed[idx] != last_pressed[idx]]
    # or
    changed = [idx for idx, (a, b) in enumerate(zip(last_pressed, pressed)) if a != b]

    print(len(changed))

    clock.tick(25)

pygame.quit()