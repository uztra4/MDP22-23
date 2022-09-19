import settings
import pygame


class Timer:
    ticks = 0
    start = 0
    end = 0

    started = 0
    ended = 0

    def __init__(self):
        pass

    @classmethod
    def print_timer(cls, screen):
        cls.calculate_ticks()
        font = pygame.freetype.SysFont(None, 34)
        font.origin = True
        millis = cls.ticks % 1000
        seconds = int(cls.ticks / 1000 % 60)
        minutes = int(cls.ticks / 60000 % 24)
        out = '{minutes:02d}:{seconds:02d}:{millis:03d}'.format(minutes=minutes, millis=millis, seconds=seconds)
        font.render_to(screen, (settings.GRID_LENGTH + 25, settings.GRID_LENGTH / 2), out, pygame.Color('dodgerblue'))

    @classmethod
    def start_timer(cls):
        cls.started = 1
        cls.ended = 0
        cls.start = pygame.time.get_ticks()

    @classmethod
    def end_timer(cls):
        cls.end = pygame.time.get_ticks()
        cls.started = 0
        cls.ended = 1

    @classmethod
    def calculate_ticks(cls):
        if cls.started:
            cls.ticks = pygame.time.get_ticks() - cls.start
        elif cls.ended:
            cls.calculate_end_ticks()

    @classmethod
    def calculate_end_ticks(cls):
        cls.ticks = cls.end - cls.start
