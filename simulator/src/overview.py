import mamba,time
from expects import *

with description('voila'):
    with it('is tested with mamba itself'):
        pass

    with it('supports Python 3'):
        pass

    with context('features'):
        with context('defining example groups'):
            with context('with arbitrary levels of nesting'):
                assert(False)

        with context('hooks'):
            with before.all:
                print('This code will be run once, before all examples in this group')

            with before.each:
                print('This code will be run once before each example in this group')

            with after.each:
                print('This code will be run once after each example in this group')

            with after.all:
                print('This code will be run once, after all examples in this group')

        with context('pending tests'):
            with _context('when running pending contexts (marked with an underscore)'):
                with it('will not run any spec under a pending context'):
                    pass

            with _it('will not run pending specs (marked with an underscore)'):
                pass



    with context('when writing assertions'):
        with it('does not include an assertion mechanism'):
            pass

        with it('works with virtually any assertion mechanism'):
            pass

        with it('can be used with expects'):
            expect(True).to(be_false)



        with it('can be used with plain assertions'):
            assert True


