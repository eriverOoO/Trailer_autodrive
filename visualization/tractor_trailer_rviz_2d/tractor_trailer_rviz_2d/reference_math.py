"""Scalar/array wrap copied from final-B tractor_trailer_model.wrap."""
import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi
