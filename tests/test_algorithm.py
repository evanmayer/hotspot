from context import algorithm as alg


def test_TestSurface_init():
    sw = (0,0)
    nw = (0,1)
    ne = (1,1)
    se = (1,0)
    surf = alg.TestSurface(sw=sw, se=se, nw=nw, ne=ne)

    # test corner assignment
    corner_list = surf.corners.reshape(4,2)
    accessor_list = [surf.sw, surf.nw, surf.se, surf.ne]
    for i, coord in enumerate([sw, nw, se, ne]):
        # underlying matrix
        assert all(a == b for a,b in zip(coord, corner_list[i])), \
            f'{coord} != {corner_list[i]}'
        # public-facing accessors
        assert all(a == b for a,b in zip(coord, accessor_list[i])), \
            f'{coord} != {accessor_list[i]}'
    
    # test origin assignment
    assert all(a == b for a,b in zip(surf.origin, sw))


def test_TestSurface_is_inbounds():
    sw = (0,0)
    nw = (0,1)
    ne = (1,1)
    se = (1,0)
    surf = alg.TestSurface(sw=sw, se=se, nw=nw, ne=ne)
    assert surf.is_inbounds((0.5, 0.5))
    assert surf.is_inbounds((0.1, 0.1))
    assert not surf.is_inbounds((0.0, 0.5))
    assert not surf.is_inbounds((1.0, 0.5))
    assert not surf.is_inbounds((1.5, 1.5))


def test_Raft_init():
    w = 2
    h = 3
    pos = (0,0)
    raft = alg.Raft(pos, w, h)

    input_list = [(-w / 2., -h / 2.), (-w / 2.,  h / 2.),
                  ( w / 2., -h / 2.), ( w / 2.,  h / 2.)]
    # test corner assignment
    corner_list = raft.corners.reshape(4,2)
    accessor_list = [raft.sw, raft.nw, raft.se, raft.ne]
    for i, coord in enumerate(input_list):
        # underlying matrix
        assert all(a == b for a,b in zip(coord, corner_list[i])), \
            f'{coord} != {corner_list[i]}'
        # public-facing accessors
        assert all(a == b for a,b in zip(coord, accessor_list[i])), \
            f'{coord} != {accessor_list[i]}'