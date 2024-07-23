// Benchmark "adder" written by ABC on Thu Jul 11 12:56:24 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n321, new_n323;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[9] ), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\b[8] ), .clkout(new_n99));
  and002aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\a[3] ), .clkout(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\b[2] ), .clkout(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n103), .b(new_n104), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  aoi012aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  oa0022aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n110));
  oai012aa1n02x5               g015(.a(new_n110), .b(new_n109), .c(new_n105), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  norp02aa1n02x5               g023(.a(new_n118), .b(new_n117), .o1(new_n119));
  nona23aa1n02x4               g024(.a(new_n111), .b(new_n119), .c(new_n116), .d(new_n100), .out0(new_n120));
  nano23aa1n02x4               g025(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n121));
  aoi112aa1n02x5               g026(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n122));
  orn002aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .o(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[6] ), .b(\b[5] ), .c(new_n123), .o1(new_n124));
  aoi112aa1n02x5               g029(.a(new_n122), .b(new_n112), .c(new_n121), .d(new_n124), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(new_n120), .b(new_n125), .o1(new_n126));
  oaoi03aa1n02x5               g031(.a(new_n98), .b(new_n99), .c(new_n126), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g033(.clk(\b[9] ), .clkout(new_n129));
  nanp02aa1n02x5               g034(.a(new_n129), .b(new_n97), .o1(new_n130));
  and002aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o(new_n131));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  160nm_ficinv00aa1n08x5       g039(.clk(new_n134), .clkout(new_n135));
  aoi112aa1n02x5               g040(.a(new_n135), .b(new_n131), .c(new_n127), .d(new_n130), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n135), .b(new_n131), .c(new_n127), .d(new_n130), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n132), .clkout(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  nano22aa1n02x4               g047(.a(new_n136), .b(new_n139), .c(new_n142), .out0(new_n143));
  nanp02aa1n02x5               g048(.a(new_n127), .b(new_n130), .o1(new_n144));
  nona22aa1n02x4               g049(.a(new_n144), .b(new_n135), .c(new_n131), .out0(new_n145));
  aoi012aa1n02x5               g050(.a(new_n142), .b(new_n145), .c(new_n139), .o1(new_n146));
  norp02aa1n02x5               g051(.a(new_n146), .b(new_n143), .o1(\s[12] ));
  oaoi13aa1n02x5               g052(.a(new_n100), .b(new_n110), .c(new_n109), .d(new_n105), .o1(new_n148));
  norp03aa1n02x5               g053(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n121), .b(new_n124), .o1(new_n150));
  nona22aa1n02x4               g055(.a(new_n150), .b(new_n122), .c(new_n112), .out0(new_n151));
  xorc02aa1n02x5               g056(.a(\a[10] ), .b(\b[9] ), .out0(new_n152));
  nano23aa1n02x4               g057(.a(new_n132), .b(new_n140), .c(new_n141), .d(new_n133), .out0(new_n153));
  xorc02aa1n02x5               g058(.a(\a[9] ), .b(\b[8] ), .out0(new_n154));
  nanp03aa1n02x5               g059(.a(new_n153), .b(new_n152), .c(new_n154), .o1(new_n155));
  160nm_ficinv00aa1n08x5       g060(.clk(new_n155), .clkout(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n151), .c(new_n148), .d(new_n149), .o1(new_n157));
  nona23aa1n02x4               g062(.a(new_n141), .b(new_n133), .c(new_n132), .d(new_n140), .out0(new_n158));
  oai022aa1n02x5               g063(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n159));
  oaib12aa1n02x5               g064(.a(new_n159), .b(new_n129), .c(\a[10] ), .out0(new_n160));
  oai012aa1n02x5               g065(.a(new_n141), .b(new_n140), .c(new_n132), .o1(new_n161));
  oai012aa1n02x5               g066(.a(new_n161), .b(new_n158), .c(new_n160), .o1(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n162), .clkout(new_n163));
  norp02aa1n02x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(new_n166));
  xobna2aa1n03x5               g071(.a(new_n166), .b(new_n157), .c(new_n163), .out0(\s[13] ));
  aoai13aa1n02x5               g072(.a(new_n163), .b(new_n155), .c(new_n120), .d(new_n125), .o1(new_n168));
  aoi012aa1n02x5               g073(.a(new_n164), .b(new_n168), .c(new_n165), .o1(new_n169));
  xnrb03aa1n02x5               g074(.a(new_n169), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nona23aa1n02x4               g077(.a(new_n172), .b(new_n165), .c(new_n164), .d(new_n171), .out0(new_n173));
  oai012aa1n02x5               g078(.a(new_n172), .b(new_n171), .c(new_n164), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n173), .c(new_n157), .d(new_n163), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  norp02aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n179), .clkout(new_n180));
  nanp02aa1n02x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  aoi122aa1n02x5               g086(.a(new_n177), .b(new_n181), .c(new_n180), .d(new_n175), .e(new_n178), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n177), .b(new_n175), .c(new_n178), .o1(new_n183));
  nanb02aa1n02x5               g088(.a(new_n179), .b(new_n181), .out0(new_n184));
  norp02aa1n02x5               g089(.a(new_n183), .b(new_n184), .o1(new_n185));
  norp02aa1n02x5               g090(.a(new_n185), .b(new_n182), .o1(\s[16] ));
  nano23aa1n02x4               g091(.a(new_n164), .b(new_n171), .c(new_n172), .d(new_n165), .out0(new_n187));
  nano23aa1n02x4               g092(.a(new_n177), .b(new_n179), .c(new_n181), .d(new_n178), .out0(new_n188));
  nano22aa1n02x4               g093(.a(new_n155), .b(new_n187), .c(new_n188), .out0(new_n189));
  aoai13aa1n02x5               g094(.a(new_n189), .b(new_n151), .c(new_n148), .d(new_n149), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(new_n177), .b(new_n178), .out0(new_n191));
  norp03aa1n02x5               g096(.a(new_n173), .b(new_n184), .c(new_n191), .o1(new_n192));
  aoi112aa1n02x5               g097(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n193));
  oai013aa1n02x4               g098(.a(new_n180), .b(new_n174), .c(new_n191), .d(new_n184), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(new_n194), .b(new_n193), .c(new_n162), .d(new_n192), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(new_n190), .b(new_n195), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g102(.clk(\a[18] ), .clkout(new_n198));
  160nm_ficinv00aa1n08x5       g103(.clk(\a[17] ), .clkout(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(\b[16] ), .clkout(new_n200));
  oaoi03aa1n02x5               g105(.a(new_n199), .b(new_n200), .c(new_n196), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[17] ), .c(new_n198), .out0(\s[18] ));
  xroi22aa1d04x5               g107(.a(new_n199), .b(\b[16] ), .c(new_n198), .d(\b[17] ), .out0(new_n203));
  nanp02aa1n02x5               g108(.a(new_n200), .b(new_n199), .o1(new_n204));
  oaoi03aa1n02x5               g109(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n205));
  norp02aa1n02x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  aoai13aa1n02x5               g113(.a(new_n208), .b(new_n205), .c(new_n196), .d(new_n203), .o1(new_n209));
  aoi112aa1n02x5               g114(.a(new_n208), .b(new_n205), .c(new_n196), .d(new_n203), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  nona22aa1n02x4               g120(.a(new_n209), .b(new_n215), .c(new_n206), .out0(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n215), .clkout(new_n217));
  oaoi13aa1n02x5               g122(.a(new_n217), .b(new_n209), .c(\a[19] ), .d(\b[18] ), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n216), .b(new_n218), .out0(\s[20] ));
  nano23aa1n02x4               g124(.a(new_n206), .b(new_n213), .c(new_n214), .d(new_n207), .out0(new_n220));
  nanp02aa1n02x5               g125(.a(new_n203), .b(new_n220), .o1(new_n221));
  oai022aa1n02x5               g126(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n222));
  oaib12aa1n02x5               g127(.a(new_n222), .b(new_n198), .c(\b[17] ), .out0(new_n223));
  nona23aa1n02x4               g128(.a(new_n214), .b(new_n207), .c(new_n206), .d(new_n213), .out0(new_n224));
  aoi012aa1n02x5               g129(.a(new_n213), .b(new_n206), .c(new_n214), .o1(new_n225));
  oai012aa1n02x5               g130(.a(new_n225), .b(new_n224), .c(new_n223), .o1(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n226), .clkout(new_n227));
  aoai13aa1n02x5               g132(.a(new_n227), .b(new_n221), .c(new_n190), .d(new_n195), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g134(.a(\b[20] ), .b(\a[21] ), .o1(new_n230));
  xorc02aa1n02x5               g135(.a(\a[21] ), .b(\b[20] ), .out0(new_n231));
  xorc02aa1n02x5               g136(.a(\a[22] ), .b(\b[21] ), .out0(new_n232));
  aoi112aa1n02x5               g137(.a(new_n230), .b(new_n232), .c(new_n228), .d(new_n231), .o1(new_n233));
  aoai13aa1n02x5               g138(.a(new_n232), .b(new_n230), .c(new_n228), .d(new_n231), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n234), .b(new_n233), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g140(.clk(\a[21] ), .clkout(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(\a[22] ), .clkout(new_n237));
  xroi22aa1d04x5               g142(.a(new_n236), .b(\b[20] ), .c(new_n237), .d(\b[21] ), .out0(new_n238));
  nanp03aa1n02x5               g143(.a(new_n238), .b(new_n203), .c(new_n220), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(\b[21] ), .clkout(new_n240));
  oaoi03aa1n02x5               g145(.a(new_n237), .b(new_n240), .c(new_n230), .o1(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n241), .clkout(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n226), .c(new_n238), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n239), .c(new_n190), .d(new_n195), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  xorc02aa1n02x5               g151(.a(\a[23] ), .b(\b[22] ), .out0(new_n247));
  xorc02aa1n02x5               g152(.a(\a[24] ), .b(\b[23] ), .out0(new_n248));
  aoi112aa1n02x5               g153(.a(new_n246), .b(new_n248), .c(new_n244), .d(new_n247), .o1(new_n249));
  aoai13aa1n02x5               g154(.a(new_n248), .b(new_n246), .c(new_n244), .d(new_n247), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n249), .out0(\s[24] ));
  and002aa1n02x5               g156(.a(new_n248), .b(new_n247), .o(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n252), .clkout(new_n253));
  nano32aa1n02x4               g158(.a(new_n253), .b(new_n238), .c(new_n203), .d(new_n220), .out0(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n225), .clkout(new_n255));
  aoai13aa1n02x5               g160(.a(new_n238), .b(new_n255), .c(new_n220), .d(new_n205), .o1(new_n256));
  norp02aa1n02x5               g161(.a(\b[23] ), .b(\a[24] ), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(\b[23] ), .b(\a[24] ), .o1(new_n258));
  aoi012aa1n02x5               g163(.a(new_n257), .b(new_n246), .c(new_n258), .o1(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n253), .c(new_n256), .d(new_n241), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n260), .c(new_n196), .d(new_n254), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(new_n261), .b(new_n260), .c(new_n196), .d(new_n254), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n262), .b(new_n263), .out0(\s[25] ));
  norp02aa1n02x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  xorc02aa1n02x5               g170(.a(\a[26] ), .b(\b[25] ), .out0(new_n266));
  nona22aa1n02x4               g171(.a(new_n262), .b(new_n266), .c(new_n265), .out0(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n265), .clkout(new_n268));
  aobi12aa1n02x5               g173(.a(new_n266), .b(new_n262), .c(new_n268), .out0(new_n269));
  norb02aa1n02x5               g174(.a(new_n267), .b(new_n269), .out0(\s[26] ));
  and002aa1n02x5               g175(.a(new_n266), .b(new_n261), .o(new_n271));
  nano22aa1n02x4               g176(.a(new_n239), .b(new_n252), .c(new_n271), .out0(new_n272));
  nanp02aa1n02x5               g177(.a(new_n196), .b(new_n272), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .carry(new_n274));
  aobi12aa1n02x5               g179(.a(new_n274), .b(new_n260), .c(new_n271), .out0(new_n275));
  xorc02aa1n02x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n275), .c(new_n273), .out0(\s[27] ));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n278), .clkout(new_n279));
  aobi12aa1n02x5               g184(.a(new_n276), .b(new_n275), .c(new_n273), .out0(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[27] ), .b(\a[28] ), .out0(new_n281));
  nano22aa1n02x4               g186(.a(new_n280), .b(new_n279), .c(new_n281), .out0(new_n282));
  aobi12aa1n02x5               g187(.a(new_n272), .b(new_n190), .c(new_n195), .out0(new_n283));
  aoai13aa1n02x5               g188(.a(new_n252), .b(new_n242), .c(new_n226), .d(new_n238), .o1(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n271), .clkout(new_n285));
  aoai13aa1n02x5               g190(.a(new_n274), .b(new_n285), .c(new_n284), .d(new_n259), .o1(new_n286));
  oai012aa1n02x5               g191(.a(new_n276), .b(new_n286), .c(new_n283), .o1(new_n287));
  aoi012aa1n02x5               g192(.a(new_n281), .b(new_n287), .c(new_n279), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n288), .b(new_n282), .o1(\s[28] ));
  norb02aa1n02x5               g194(.a(new_n276), .b(new_n281), .out0(new_n290));
  aobi12aa1n02x5               g195(.a(new_n290), .b(new_n275), .c(new_n273), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n291), .b(new_n292), .c(new_n293), .out0(new_n294));
  oai012aa1n02x5               g199(.a(new_n290), .b(new_n286), .c(new_n283), .o1(new_n295));
  aoi012aa1n02x5               g200(.a(new_n293), .b(new_n295), .c(new_n292), .o1(new_n296));
  norp02aa1n02x5               g201(.a(new_n296), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n276), .b(new_n293), .c(new_n281), .out0(new_n299));
  aobi12aa1n02x5               g204(.a(new_n299), .b(new_n275), .c(new_n273), .out0(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  oai012aa1n02x5               g208(.a(new_n299), .b(new_n286), .c(new_n283), .o1(new_n304));
  aoi012aa1n02x5               g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n305), .b(new_n303), .o1(\s[30] ));
  norb02aa1n02x5               g211(.a(new_n299), .b(new_n302), .out0(new_n307));
  aobi12aa1n02x5               g212(.a(new_n307), .b(new_n275), .c(new_n273), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nano22aa1n02x4               g215(.a(new_n308), .b(new_n309), .c(new_n310), .out0(new_n311));
  oai012aa1n02x5               g216(.a(new_n307), .b(new_n286), .c(new_n283), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n310), .b(new_n312), .c(new_n309), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnbna2aa1n03x5               g219(.a(new_n109), .b(new_n103), .c(new_n104), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n148), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nona22aa1n02x4               g223(.a(new_n111), .b(new_n118), .c(new_n100), .out0(new_n319));
  xobna2aa1n03x5               g224(.a(new_n117), .b(new_n319), .c(new_n123), .out0(\s[6] ));
  160nm_fiao0012aa1n02p5x5     g225(.a(new_n124), .b(new_n148), .c(new_n119), .o(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g227(.a(new_n114), .b(new_n321), .c(new_n115), .o1(new_n323));
  xnrb03aa1n02x5               g228(.a(new_n323), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g229(.a(new_n154), .b(new_n120), .c(new_n125), .out0(\s[9] ));
endmodule


