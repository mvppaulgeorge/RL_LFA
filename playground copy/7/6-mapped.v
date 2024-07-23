// Benchmark "adder" written by ABC on Thu Jul 11 11:22:12 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n151, new_n152, new_n153, new_n154,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n333, new_n336, new_n338, new_n340;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[9] ), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\b[8] ), .clkout(new_n99));
  nanp02aa1n02x5               g004(.a(new_n99), .b(new_n98), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  norp02aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nanb02aa1n02x5               g012(.a(new_n106), .b(new_n107), .out0(new_n108));
  xnrc02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .out0(new_n109));
  norp03aa1n02x5               g014(.a(new_n105), .b(new_n108), .c(new_n109), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  aoi012aa1n02x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nona23aa1n02x4               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  160nm_fiao0012aa1n02p5x5     g024(.a(new_n115), .b(new_n117), .c(new_n116), .o(new_n120));
  oabi12aa1n02x5               g025(.a(new_n120), .b(new_n119), .c(new_n114), .out0(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(new_n101), .clkout(new_n122));
  nanp02aa1n02x5               g027(.a(new_n103), .b(new_n102), .o1(new_n123));
  norp02aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n107), .b(new_n124), .c(new_n106), .o1(new_n125));
  oai112aa1n02x5               g030(.a(new_n122), .b(new_n123), .c(new_n105), .d(new_n125), .o1(new_n126));
  xorc02aa1n02x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n126), .c(new_n121), .d(new_n110), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n100), .out0(\s[10] ));
  nanb02aa1n02x5               g034(.a(new_n101), .b(new_n102), .out0(new_n130));
  nanb02aa1n02x5               g035(.a(new_n103), .b(new_n104), .out0(new_n131));
  norp02aa1n02x5               g036(.a(new_n109), .b(new_n108), .o1(new_n132));
  nona22aa1n02x4               g037(.a(new_n132), .b(new_n131), .c(new_n130), .out0(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n114), .clkout(new_n134));
  nano23aa1n02x4               g039(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n116), .out0(new_n135));
  aoi012aa1n02x5               g040(.a(new_n120), .b(new_n135), .c(new_n134), .o1(new_n136));
  norp03aa1n02x5               g041(.a(new_n125), .b(new_n131), .c(new_n130), .o1(new_n137));
  nano22aa1n02x4               g042(.a(new_n137), .b(new_n122), .c(new_n123), .out0(new_n138));
  oai012aa1n02x5               g043(.a(new_n138), .b(new_n136), .c(new_n133), .o1(new_n139));
  norp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  160nm_ficinv00aa1n08x5       g047(.clk(\a[10] ), .clkout(new_n143));
  oai022aa1n02x5               g048(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n144));
  oaib12aa1n02x5               g049(.a(new_n144), .b(new_n143), .c(\b[9] ), .out0(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(new_n145), .clkout(new_n146));
  xroi22aa1d04x5               g051(.a(new_n143), .b(\b[9] ), .c(new_n99), .d(\a[9] ), .out0(new_n147));
  aoai13aa1n02x5               g052(.a(new_n142), .b(new_n146), .c(new_n139), .d(new_n147), .o1(new_n148));
  aoi112aa1n02x5               g053(.a(new_n146), .b(new_n142), .c(new_n139), .d(new_n147), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n148), .b(new_n149), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n140), .clkout(new_n151));
  norp02aa1n02x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n148), .c(new_n151), .out0(\s[12] ));
  nano23aa1n02x4               g060(.a(new_n140), .b(new_n152), .c(new_n153), .d(new_n141), .out0(new_n156));
  nanp03aa1n02x5               g061(.a(new_n156), .b(new_n97), .c(new_n127), .o1(new_n157));
  160nm_ficinv00aa1n08x5       g062(.clk(new_n157), .clkout(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n126), .c(new_n121), .d(new_n110), .o1(new_n159));
  nona23aa1n02x4               g064(.a(new_n153), .b(new_n141), .c(new_n140), .d(new_n152), .out0(new_n160));
  aoi012aa1n02x5               g065(.a(new_n152), .b(new_n140), .c(new_n153), .o1(new_n161));
  oai012aa1n02x5               g066(.a(new_n161), .b(new_n160), .c(new_n145), .o1(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n162), .clkout(new_n163));
  norp02aa1n02x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(new_n166));
  xobna2aa1n03x5               g071(.a(new_n166), .b(new_n159), .c(new_n163), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n164), .clkout(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n166), .c(new_n159), .d(new_n163), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nona23aa1n02x4               g077(.a(new_n172), .b(new_n165), .c(new_n164), .d(new_n171), .out0(new_n173));
  oaoi03aa1n02x5               g078(.a(\a[14] ), .b(\b[13] ), .c(new_n168), .o1(new_n174));
  160nm_ficinv00aa1n08x5       g079(.clk(new_n174), .clkout(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n173), .c(new_n159), .d(new_n163), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  xorc02aa1n02x5               g083(.a(\a[15] ), .b(\b[14] ), .out0(new_n179));
  xorc02aa1n02x5               g084(.a(\a[16] ), .b(\b[15] ), .out0(new_n180));
  aoi112aa1n02x5               g085(.a(new_n180), .b(new_n178), .c(new_n176), .d(new_n179), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n180), .b(new_n178), .c(new_n176), .d(new_n179), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n182), .b(new_n181), .out0(\s[16] ));
  aoi012aa1n02x5               g088(.a(new_n126), .b(new_n121), .c(new_n110), .o1(new_n184));
  xnrc02aa1n02x5               g089(.a(\b[14] ), .b(\a[15] ), .out0(new_n185));
  xnrc02aa1n02x5               g090(.a(\b[15] ), .b(\a[16] ), .out0(new_n186));
  norp03aa1n02x5               g091(.a(new_n173), .b(new_n186), .c(new_n185), .o1(new_n187));
  nanp03aa1n02x5               g092(.a(new_n187), .b(new_n147), .c(new_n156), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n180), .b(new_n179), .o1(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n178), .clkout(new_n190));
  oaoi03aa1n02x5               g095(.a(\a[16] ), .b(\b[15] ), .c(new_n190), .o1(new_n191));
  oabi12aa1n02x5               g096(.a(new_n191), .b(new_n189), .c(new_n175), .out0(new_n192));
  aoi012aa1n02x5               g097(.a(new_n192), .b(new_n162), .c(new_n187), .o1(new_n193));
  oai012aa1n02x5               g098(.a(new_n193), .b(new_n184), .c(new_n188), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g100(.clk(\a[18] ), .clkout(new_n196));
  160nm_ficinv00aa1n08x5       g101(.clk(\a[17] ), .clkout(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(\b[16] ), .clkout(new_n198));
  oaoi03aa1n02x5               g103(.a(new_n197), .b(new_n198), .c(new_n194), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[17] ), .c(new_n196), .out0(\s[18] ));
  norp03aa1n02x5               g105(.a(new_n157), .b(new_n173), .c(new_n189), .o1(new_n201));
  aoai13aa1n02x5               g106(.a(new_n201), .b(new_n126), .c(new_n110), .d(new_n121), .o1(new_n202));
  xroi22aa1d04x5               g107(.a(new_n197), .b(\b[16] ), .c(new_n196), .d(\b[17] ), .out0(new_n203));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n203), .clkout(new_n204));
  oai022aa1n02x5               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  oaib12aa1n02x5               g110(.a(new_n205), .b(new_n196), .c(\b[17] ), .out0(new_n206));
  aoai13aa1n02x5               g111(.a(new_n206), .b(new_n204), .c(new_n202), .d(new_n193), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(\b[19] ), .clkout(new_n214));
  nanb02aa1n02x5               g119(.a(\a[20] ), .b(new_n214), .out0(new_n215));
  nanp02aa1n02x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(new_n215), .b(new_n216), .o1(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(new_n217), .clkout(new_n218));
  aoi112aa1n02x5               g123(.a(new_n210), .b(new_n218), .c(new_n207), .d(new_n213), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n210), .clkout(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(new_n206), .clkout(new_n221));
  aoai13aa1n02x5               g126(.a(new_n213), .b(new_n221), .c(new_n194), .d(new_n203), .o1(new_n222));
  aoi012aa1n02x5               g127(.a(new_n217), .b(new_n222), .c(new_n220), .o1(new_n223));
  norp02aa1n02x5               g128(.a(new_n223), .b(new_n219), .o1(\s[20] ));
  norp02aa1n02x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nona23aa1n02x4               g130(.a(new_n216), .b(new_n211), .c(new_n210), .d(new_n225), .out0(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n226), .clkout(new_n227));
  nanp02aa1n02x5               g132(.a(new_n203), .b(new_n227), .o1(new_n228));
  nanp02aa1n02x5               g133(.a(new_n210), .b(new_n216), .o1(new_n229));
  norp03aa1n02x5               g134(.a(new_n206), .b(new_n212), .c(new_n217), .o1(new_n230));
  nano22aa1n02x4               g135(.a(new_n230), .b(new_n215), .c(new_n229), .out0(new_n231));
  aoai13aa1n02x5               g136(.a(new_n231), .b(new_n228), .c(new_n202), .d(new_n193), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  xnrc02aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .out0(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  xnrc02aa1n02x5               g141(.a(\b[21] ), .b(\a[22] ), .out0(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n237), .clkout(new_n238));
  aoi112aa1n02x5               g143(.a(new_n234), .b(new_n238), .c(new_n232), .d(new_n236), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n234), .clkout(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n228), .clkout(new_n241));
  oai112aa1n02x5               g146(.a(new_n229), .b(new_n215), .c(new_n226), .d(new_n206), .o1(new_n242));
  aoai13aa1n02x5               g147(.a(new_n236), .b(new_n242), .c(new_n194), .d(new_n241), .o1(new_n243));
  aoi012aa1n02x5               g148(.a(new_n237), .b(new_n243), .c(new_n240), .o1(new_n244));
  norp02aa1n02x5               g149(.a(new_n244), .b(new_n239), .o1(\s[22] ));
  norp02aa1n02x5               g150(.a(new_n237), .b(new_n235), .o1(new_n246));
  oaoi03aa1n02x5               g151(.a(\a[22] ), .b(\b[21] ), .c(new_n240), .o1(new_n247));
  aoi012aa1n02x5               g152(.a(new_n247), .b(new_n242), .c(new_n246), .o1(new_n248));
  nanp03aa1n02x5               g153(.a(new_n203), .b(new_n227), .c(new_n246), .o1(new_n249));
  aoai13aa1n02x5               g154(.a(new_n248), .b(new_n249), .c(new_n202), .d(new_n193), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[23] ), .b(\b[22] ), .out0(new_n253));
  xorc02aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .out0(new_n254));
  aoi112aa1n02x5               g159(.a(new_n252), .b(new_n254), .c(new_n250), .d(new_n253), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n252), .clkout(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n248), .clkout(new_n257));
  160nm_ficinv00aa1n08x5       g162(.clk(new_n249), .clkout(new_n258));
  aoai13aa1n02x5               g163(.a(new_n253), .b(new_n257), .c(new_n194), .d(new_n258), .o1(new_n259));
  aobi12aa1n02x5               g164(.a(new_n254), .b(new_n259), .c(new_n256), .out0(new_n260));
  norp02aa1n02x5               g165(.a(new_n260), .b(new_n255), .o1(\s[24] ));
  nano32aa1n02x4               g166(.a(new_n228), .b(new_n254), .c(new_n246), .d(new_n253), .out0(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(new_n262), .clkout(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[22] ), .b(\a[23] ), .out0(new_n264));
  norb02aa1n02x5               g169(.a(new_n254), .b(new_n264), .out0(new_n265));
  norp02aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .o1(new_n266));
  aoi112aa1n02x5               g171(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n267));
  nanp03aa1n02x5               g172(.a(new_n247), .b(new_n253), .c(new_n254), .o1(new_n268));
  nona22aa1n02x4               g173(.a(new_n268), .b(new_n267), .c(new_n266), .out0(new_n269));
  aoi013aa1n02x4               g174(.a(new_n269), .b(new_n242), .c(new_n246), .d(new_n265), .o1(new_n270));
  aoai13aa1n02x5               g175(.a(new_n270), .b(new_n263), .c(new_n202), .d(new_n193), .o1(new_n271));
  xorb03aa1n02x5               g176(.a(new_n271), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g177(.a(\b[24] ), .b(\a[25] ), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[25] ), .b(\b[24] ), .out0(new_n274));
  xorc02aa1n02x5               g179(.a(\a[26] ), .b(\b[25] ), .out0(new_n275));
  aoi112aa1n02x5               g180(.a(new_n273), .b(new_n275), .c(new_n271), .d(new_n274), .o1(new_n276));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n273), .clkout(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n270), .clkout(new_n278));
  aoai13aa1n02x5               g183(.a(new_n274), .b(new_n278), .c(new_n194), .d(new_n262), .o1(new_n279));
  aobi12aa1n02x5               g184(.a(new_n275), .b(new_n279), .c(new_n277), .out0(new_n280));
  norp02aa1n02x5               g185(.a(new_n280), .b(new_n276), .o1(\s[26] ));
  aoi013aa1n02x4               g186(.a(new_n191), .b(new_n174), .c(new_n180), .d(new_n179), .o1(new_n282));
  aob012aa1n02x5               g187(.a(new_n282), .b(new_n162), .c(new_n187), .out0(new_n283));
  and002aa1n02x5               g188(.a(new_n275), .b(new_n274), .o(new_n284));
  nano22aa1n02x4               g189(.a(new_n249), .b(new_n284), .c(new_n265), .out0(new_n285));
  aoai13aa1n02x5               g190(.a(new_n285), .b(new_n283), .c(new_n139), .d(new_n201), .o1(new_n286));
  nano22aa1n02x4               g191(.a(new_n231), .b(new_n246), .c(new_n265), .out0(new_n287));
  oao003aa1n02x5               g192(.a(\a[26] ), .b(\b[25] ), .c(new_n277), .carry(new_n288));
  160nm_ficinv00aa1n08x5       g193(.clk(new_n288), .clkout(new_n289));
  oaoi13aa1n02x5               g194(.a(new_n289), .b(new_n284), .c(new_n287), .d(new_n269), .o1(new_n290));
  norp02aa1n02x5               g195(.a(\b[26] ), .b(\a[27] ), .o1(new_n291));
  nanp02aa1n02x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  norb02aa1n02x5               g197(.a(new_n292), .b(new_n291), .out0(new_n293));
  xnbna2aa1n03x5               g198(.a(new_n293), .b(new_n290), .c(new_n286), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g199(.clk(new_n291), .clkout(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[27] ), .b(\a[28] ), .out0(new_n296));
  160nm_ficinv00aa1n08x5       g201(.clk(new_n269), .clkout(new_n297));
  nanp02aa1n02x5               g202(.a(new_n254), .b(new_n253), .o1(new_n298));
  nona32aa1n02x4               g203(.a(new_n242), .b(new_n298), .c(new_n237), .d(new_n235), .out0(new_n299));
  160nm_ficinv00aa1n08x5       g204(.clk(new_n284), .clkout(new_n300));
  aoai13aa1n02x5               g205(.a(new_n288), .b(new_n300), .c(new_n299), .d(new_n297), .o1(new_n301));
  aoai13aa1n02x5               g206(.a(new_n292), .b(new_n301), .c(new_n194), .d(new_n285), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n296), .b(new_n302), .c(new_n295), .o1(new_n303));
  aoi022aa1n02x5               g208(.a(new_n290), .b(new_n286), .c(\a[27] ), .d(\b[26] ), .o1(new_n304));
  nano22aa1n02x4               g209(.a(new_n304), .b(new_n295), .c(new_n296), .out0(new_n305));
  norp02aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[28] ));
  nano22aa1n02x4               g211(.a(new_n296), .b(new_n295), .c(new_n292), .out0(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n301), .c(new_n194), .d(new_n285), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[28] ), .b(\b[27] ), .c(new_n295), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[28] ), .b(\a[29] ), .out0(new_n310));
  aoi012aa1n02x5               g215(.a(new_n310), .b(new_n308), .c(new_n309), .o1(new_n311));
  aobi12aa1n02x5               g216(.a(new_n307), .b(new_n290), .c(new_n286), .out0(new_n312));
  nano22aa1n02x4               g217(.a(new_n312), .b(new_n309), .c(new_n310), .out0(new_n313));
  norp02aa1n02x5               g218(.a(new_n311), .b(new_n313), .o1(\s[29] ));
  xorb03aa1n02x5               g219(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g220(.a(new_n293), .b(new_n310), .c(new_n296), .out0(new_n316));
  aoai13aa1n02x5               g221(.a(new_n316), .b(new_n301), .c(new_n194), .d(new_n285), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[29] ), .b(\b[28] ), .c(new_n309), .carry(new_n318));
  xnrc02aa1n02x5               g223(.a(\b[29] ), .b(\a[30] ), .out0(new_n319));
  aoi012aa1n02x5               g224(.a(new_n319), .b(new_n317), .c(new_n318), .o1(new_n320));
  aobi12aa1n02x5               g225(.a(new_n316), .b(new_n290), .c(new_n286), .out0(new_n321));
  nano22aa1n02x4               g226(.a(new_n321), .b(new_n318), .c(new_n319), .out0(new_n322));
  norp02aa1n02x5               g227(.a(new_n320), .b(new_n322), .o1(\s[30] ));
  xnrc02aa1n02x5               g228(.a(\b[30] ), .b(\a[31] ), .out0(new_n324));
  norb03aa1n02x5               g229(.a(new_n307), .b(new_n319), .c(new_n310), .out0(new_n325));
  aobi12aa1n02x5               g230(.a(new_n325), .b(new_n290), .c(new_n286), .out0(new_n326));
  oao003aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .c(new_n318), .carry(new_n327));
  nano22aa1n02x4               g232(.a(new_n326), .b(new_n324), .c(new_n327), .out0(new_n328));
  aoai13aa1n02x5               g233(.a(new_n325), .b(new_n301), .c(new_n194), .d(new_n285), .o1(new_n329));
  aoi012aa1n02x5               g234(.a(new_n324), .b(new_n329), .c(new_n327), .o1(new_n330));
  norp02aa1n02x5               g235(.a(new_n330), .b(new_n328), .o1(\s[31] ));
  xnrb03aa1n02x5               g236(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g237(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n333));
  xorb03aa1n02x5               g238(.a(new_n333), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g239(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g240(.a(\a[5] ), .b(\b[4] ), .c(new_n136), .o1(new_n336));
  xorb03aa1n02x5               g241(.a(new_n336), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aobi12aa1n02x5               g242(.a(new_n125), .b(new_n121), .c(new_n132), .out0(new_n338));
  xnrb03aa1n02x5               g243(.a(new_n338), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g244(.a(\a[7] ), .b(\b[6] ), .c(new_n338), .o1(new_n340));
  xorb03aa1n02x5               g245(.a(new_n340), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g246(.a(new_n184), .b(\b[8] ), .c(new_n98), .out0(\s[9] ));
endmodule


