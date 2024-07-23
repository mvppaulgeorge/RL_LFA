// Benchmark "adder" written by ABC on Thu Jul 11 11:37:16 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n317, new_n319, new_n320,
    new_n321, new_n322, new_n324, new_n325, new_n326;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(new_n98), .clkout(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  xnrc02aa1n02x5               g011(.a(\b[5] ), .b(\a[6] ), .out0(new_n107));
  xnrc02aa1n02x5               g012(.a(\b[4] ), .b(\a[5] ), .out0(new_n108));
  norp03aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  nanp02aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  oai012aa1n02x5               g022(.a(new_n115), .b(new_n117), .c(new_n116), .o1(new_n118));
  160nm_fiao0012aa1n02p5x5     g023(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n119));
  oabi12aa1n02x5               g024(.a(new_n119), .b(new_n114), .c(new_n118), .out0(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(new_n102), .clkout(new_n121));
  nanp02aa1n02x5               g026(.a(new_n104), .b(new_n103), .o1(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\b[5] ), .clkout(new_n123));
  oai022aa1n02x5               g028(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  oaib12aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(\a[6] ), .out0(new_n125));
  oai112aa1n02x5               g030(.a(new_n121), .b(new_n122), .c(new_n106), .d(new_n125), .o1(new_n126));
  aoai13aa1n02x5               g031(.a(new_n101), .b(new_n126), .c(new_n120), .d(new_n109), .o1(new_n127));
  obai22aa1n02x7               g032(.a(new_n127), .b(new_n100), .c(new_n97), .d(new_n99), .out0(new_n128));
  nona32aa1n02x4               g033(.a(new_n127), .b(new_n100), .c(new_n99), .d(new_n97), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n128), .b(new_n129), .o1(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n129), .c(new_n98), .out0(\s[11] ));
  aoi013aa1n02x4               g039(.a(new_n131), .b(new_n129), .c(new_n98), .d(new_n132), .o1(new_n135));
  norp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n136), .clkout(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n135), .b(new_n138), .c(new_n137), .out0(\s[12] ));
  norb02aa1n02x5               g044(.a(new_n132), .b(new_n131), .out0(new_n140));
  norb02aa1n02x5               g045(.a(new_n138), .b(new_n136), .out0(new_n141));
  nano23aa1n02x4               g046(.a(new_n97), .b(new_n100), .c(new_n101), .d(new_n98), .out0(new_n142));
  and003aa1n02x5               g047(.a(new_n142), .b(new_n141), .c(new_n140), .o(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n126), .c(new_n120), .d(new_n109), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(new_n131), .b(new_n138), .o1(new_n145));
  nanb02aa1n02x5               g050(.a(new_n136), .b(new_n138), .out0(new_n146));
  oai022aa1n02x5               g051(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n147));
  nano23aa1n02x4               g052(.a(new_n146), .b(new_n133), .c(new_n147), .d(new_n98), .out0(new_n148));
  nano22aa1n02x4               g053(.a(new_n148), .b(new_n137), .c(new_n145), .out0(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(\a[13] ), .clkout(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(\b[12] ), .clkout(new_n151));
  nanp02aa1n02x5               g056(.a(new_n151), .b(new_n150), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n152), .b(new_n153), .o1(new_n154));
  xobna2aa1n03x5               g059(.a(new_n154), .b(new_n144), .c(new_n149), .out0(\s[13] ));
  aoai13aa1n02x5               g060(.a(new_n152), .b(new_n154), .c(new_n144), .d(new_n149), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nanp02aa1n02x5               g062(.a(new_n144), .b(new_n149), .o1(new_n158));
  norp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  norp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nanb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(new_n164));
  norp02aa1n02x5               g069(.a(new_n164), .b(new_n154), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n163), .b(new_n162), .c(new_n150), .d(new_n151), .o1(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n166), .clkout(new_n167));
  aoai13aa1n02x5               g072(.a(new_n161), .b(new_n167), .c(new_n158), .d(new_n165), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n161), .b(new_n167), .c(new_n158), .d(new_n165), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(\s[15] ));
  norp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  nona22aa1n02x4               g078(.a(new_n168), .b(new_n173), .c(new_n159), .out0(new_n174));
  160nm_ficinv00aa1n08x5       g079(.clk(new_n173), .clkout(new_n175));
  oaoi13aa1n02x5               g080(.a(new_n175), .b(new_n168), .c(\a[15] ), .d(\b[14] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n174), .b(new_n176), .out0(\s[16] ));
  nano23aa1n02x4               g082(.a(new_n159), .b(new_n171), .c(new_n172), .d(new_n160), .out0(new_n178));
  nona22aa1n02x4               g083(.a(new_n178), .b(new_n164), .c(new_n154), .out0(new_n179));
  nano32aa1n02x4               g084(.a(new_n179), .b(new_n142), .c(new_n141), .d(new_n140), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n126), .c(new_n120), .d(new_n109), .o1(new_n181));
  norp02aa1n02x5               g086(.a(new_n100), .b(new_n97), .o1(new_n182));
  nona23aa1n02x4               g087(.a(new_n141), .b(new_n140), .c(new_n182), .d(new_n99), .out0(new_n183));
  nanp03aa1n02x5               g088(.a(new_n183), .b(new_n137), .c(new_n145), .o1(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(new_n179), .clkout(new_n185));
  nona23aa1n02x4               g090(.a(new_n172), .b(new_n160), .c(new_n159), .d(new_n171), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n159), .b(new_n172), .o1(new_n187));
  oai122aa1n02x7               g092(.a(new_n187), .b(new_n186), .c(new_n166), .d(\b[15] ), .e(\a[16] ), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n188), .b(new_n184), .c(new_n185), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(new_n189), .b(new_n181), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g096(.clk(\a[18] ), .clkout(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(\a[17] ), .clkout(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(\b[16] ), .clkout(new_n194));
  oaoi03aa1n02x5               g099(.a(new_n193), .b(new_n194), .c(new_n190), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[17] ), .c(new_n192), .out0(\s[18] ));
  xroi22aa1d04x5               g101(.a(new_n193), .b(\b[16] ), .c(new_n192), .d(\b[17] ), .out0(new_n197));
  nanp02aa1n02x5               g102(.a(new_n194), .b(new_n193), .o1(new_n198));
  oaoi03aa1n02x5               g103(.a(\a[18] ), .b(\b[17] ), .c(new_n198), .o1(new_n199));
  norp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n199), .c(new_n190), .d(new_n197), .o1(new_n203));
  aoi112aa1n02x5               g108(.a(new_n202), .b(new_n199), .c(new_n190), .d(new_n197), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  nona22aa1n02x4               g114(.a(new_n203), .b(new_n209), .c(new_n200), .out0(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n209), .clkout(new_n211));
  oaoi13aa1n02x5               g116(.a(new_n211), .b(new_n203), .c(\a[19] ), .d(\b[18] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n210), .b(new_n212), .out0(\s[20] ));
  nano23aa1n02x4               g118(.a(new_n200), .b(new_n207), .c(new_n208), .d(new_n201), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n197), .b(new_n214), .o1(new_n215));
  oai022aa1n02x5               g120(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n216));
  oaib12aa1n02x5               g121(.a(new_n216), .b(new_n192), .c(\b[17] ), .out0(new_n217));
  nona23aa1n02x4               g122(.a(new_n208), .b(new_n201), .c(new_n200), .d(new_n207), .out0(new_n218));
  aoi012aa1n02x5               g123(.a(new_n207), .b(new_n200), .c(new_n208), .o1(new_n219));
  oai012aa1n02x5               g124(.a(new_n219), .b(new_n218), .c(new_n217), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n220), .clkout(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n215), .c(new_n189), .d(new_n181), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xorc02aa1n02x5               g129(.a(\a[21] ), .b(\b[20] ), .out0(new_n225));
  xorc02aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .out0(new_n226));
  aoi112aa1n02x5               g131(.a(new_n224), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n226), .b(new_n224), .c(new_n222), .d(new_n225), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[21] ), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(\a[22] ), .clkout(new_n231));
  xroi22aa1d04x5               g136(.a(new_n230), .b(\b[20] ), .c(new_n231), .d(\b[21] ), .out0(new_n232));
  nanp03aa1n02x5               g137(.a(new_n232), .b(new_n197), .c(new_n214), .o1(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\b[21] ), .clkout(new_n234));
  oaoi03aa1n02x5               g139(.a(new_n231), .b(new_n234), .c(new_n224), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  aoi012aa1n02x5               g141(.a(new_n236), .b(new_n220), .c(new_n232), .o1(new_n237));
  aoai13aa1n02x5               g142(.a(new_n237), .b(new_n233), .c(new_n189), .d(new_n181), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  xorc02aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .out0(new_n242));
  aoi112aa1n02x5               g147(.a(new_n240), .b(new_n242), .c(new_n238), .d(new_n241), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n242), .b(new_n240), .c(new_n238), .d(new_n241), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(\s[24] ));
  and002aa1n02x5               g150(.a(new_n242), .b(new_n241), .o(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  nano32aa1n02x4               g152(.a(new_n247), .b(new_n232), .c(new_n197), .d(new_n214), .out0(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n219), .clkout(new_n249));
  aoai13aa1n02x5               g154(.a(new_n232), .b(new_n249), .c(new_n214), .d(new_n199), .o1(new_n250));
  orn002aa1n02x5               g155(.a(\a[23] ), .b(\b[22] ), .o(new_n251));
  oao003aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .c(new_n251), .carry(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n247), .c(new_n250), .d(new_n235), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n253), .c(new_n190), .d(new_n248), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(new_n254), .b(new_n253), .c(new_n190), .d(new_n248), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n255), .b(new_n256), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  nona22aa1n02x4               g164(.a(new_n255), .b(new_n259), .c(new_n258), .out0(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n258), .clkout(new_n261));
  aobi12aa1n02x5               g166(.a(new_n259), .b(new_n255), .c(new_n261), .out0(new_n262));
  norb02aa1n02x5               g167(.a(new_n260), .b(new_n262), .out0(\s[26] ));
  nanp02aa1n02x5               g168(.a(new_n120), .b(new_n109), .o1(new_n264));
  nanb02aa1n02x5               g169(.a(new_n126), .b(new_n264), .out0(new_n265));
  oabi12aa1n02x5               g170(.a(new_n188), .b(new_n149), .c(new_n179), .out0(new_n266));
  and002aa1n02x5               g171(.a(new_n259), .b(new_n254), .o(new_n267));
  nano22aa1n02x4               g172(.a(new_n233), .b(new_n246), .c(new_n267), .out0(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n266), .c(new_n265), .d(new_n180), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n270));
  aobi12aa1n02x5               g175(.a(new_n270), .b(new_n253), .c(new_n267), .out0(new_n271));
  xorc02aa1n02x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnbna2aa1n03x5               g177(.a(new_n272), .b(new_n271), .c(new_n269), .out0(\s[27] ));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n274), .clkout(new_n275));
  aobi12aa1n02x5               g180(.a(new_n272), .b(new_n271), .c(new_n269), .out0(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  nano22aa1n02x4               g182(.a(new_n276), .b(new_n275), .c(new_n277), .out0(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n268), .clkout(new_n279));
  aoi012aa1n02x5               g184(.a(new_n279), .b(new_n189), .c(new_n181), .o1(new_n280));
  aoai13aa1n02x5               g185(.a(new_n246), .b(new_n236), .c(new_n220), .d(new_n232), .o1(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n267), .clkout(new_n282));
  aoai13aa1n02x5               g187(.a(new_n270), .b(new_n282), .c(new_n281), .d(new_n252), .o1(new_n283));
  oai012aa1n02x5               g188(.a(new_n272), .b(new_n283), .c(new_n280), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n277), .b(new_n284), .c(new_n275), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n285), .b(new_n278), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n272), .b(new_n277), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n287), .b(new_n283), .c(new_n280), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n02x5               g196(.a(new_n287), .b(new_n271), .c(new_n269), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n02x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n272), .b(new_n290), .c(new_n277), .out0(new_n296));
  oai012aa1n02x5               g201(.a(new_n296), .b(new_n283), .c(new_n280), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  aoi012aa1n02x5               g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n02x5               g205(.a(new_n296), .b(new_n271), .c(new_n269), .out0(new_n301));
  nano22aa1n02x4               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n02x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  norb02aa1n02x5               g208(.a(new_n296), .b(new_n299), .out0(new_n304));
  aobi12aa1n02x5               g209(.a(new_n304), .b(new_n271), .c(new_n269), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nano22aa1n02x4               g212(.a(new_n305), .b(new_n306), .c(new_n307), .out0(new_n308));
  oai012aa1n02x5               g213(.a(new_n304), .b(new_n283), .c(new_n280), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  norp02aa1n02x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n118), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n118), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n120), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g220(.a(new_n108), .b(new_n120), .out0(new_n316));
  oai012aa1n02x5               g221(.a(new_n316), .b(\b[4] ), .c(\a[5] ), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g223(.a(new_n104), .b(new_n105), .out0(new_n319));
  nanb02aa1n02x5               g224(.a(new_n107), .b(new_n317), .out0(new_n320));
  oaoi13aa1n02x5               g225(.a(new_n319), .b(new_n320), .c(\a[6] ), .d(\b[5] ), .o1(new_n321));
  oai112aa1n02x5               g226(.a(new_n320), .b(new_n319), .c(\b[5] ), .d(\a[6] ), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(\s[7] ));
  nanb02aa1n02x5               g228(.a(new_n102), .b(new_n103), .out0(new_n324));
  oab012aa1n02x4               g229(.a(new_n324), .b(new_n321), .c(new_n104), .out0(new_n325));
  aoi112aa1n02x5               g230(.a(new_n321), .b(new_n104), .c(new_n121), .d(new_n103), .o1(new_n326));
  norp02aa1n02x5               g231(.a(new_n325), .b(new_n326), .o1(\s[8] ));
  xorb03aa1n02x5               g232(.a(new_n265), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


