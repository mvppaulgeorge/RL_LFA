// Benchmark "adder" written by ABC on Thu Jul 11 12:15:06 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n229, new_n230,
    new_n231, new_n232, new_n233, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n243, new_n244, new_n245, new_n246,
    new_n247, new_n249, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n314, new_n316, new_n317, new_n319;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  and002aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o(new_n98));
  norp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  orn002aa1n02x5               g004(.a(\a[9] ), .b(\b[8] ), .o(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\a[2] ), .clkout(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\b[1] ), .clkout(new_n102));
  nanp02aa1n02x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  oaoi03aa1n02x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nona23aa1n02x4               g013(.a(new_n108), .b(new_n106), .c(new_n105), .d(new_n107), .out0(new_n109));
  aoi012aa1n02x5               g014(.a(new_n105), .b(new_n107), .c(new_n106), .o1(new_n110));
  oai012aa1n02x5               g015(.a(new_n110), .b(new_n109), .c(new_n104), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  xorc02aa1n02x5               g021(.a(\a[6] ), .b(\b[5] ), .out0(new_n117));
  xorc02aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .out0(new_n118));
  nano22aa1n02x4               g023(.a(new_n116), .b(new_n117), .c(new_n118), .out0(new_n119));
  160nm_ficinv00aa1n08x5       g024(.clk(\a[6] ), .clkout(new_n120));
  160nm_ficinv00aa1n08x5       g025(.clk(\b[5] ), .clkout(new_n121));
  norp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(new_n120), .b(new_n121), .c(new_n122), .o1(new_n123));
  oai012aa1n02x5               g028(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n124), .b(new_n116), .c(new_n123), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n119), .d(new_n111), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n99), .b(new_n127), .c(new_n100), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g033(.clk(new_n97), .clkout(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n98), .c(new_n127), .d(new_n100), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  norp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoi112aa1n02x5               g042(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n134), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n134), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(\s[12] ));
  nanp02aa1n02x5               g045(.a(new_n119), .b(new_n111), .o1(new_n141));
  160nm_ficinv00aa1n08x5       g046(.clk(new_n125), .clkout(new_n142));
  nano23aa1n02x4               g047(.a(new_n132), .b(new_n135), .c(new_n136), .d(new_n133), .out0(new_n143));
  nanp03aa1n02x5               g048(.a(new_n143), .b(new_n99), .c(new_n126), .o1(new_n144));
  oaoi03aa1n02x5               g049(.a(\a[10] ), .b(\b[9] ), .c(new_n100), .o1(new_n145));
  oai012aa1n02x5               g050(.a(new_n136), .b(new_n135), .c(new_n132), .o1(new_n146));
  aobi12aa1n02x5               g051(.a(new_n146), .b(new_n143), .c(new_n145), .out0(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n144), .c(new_n141), .d(new_n142), .o1(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  orn002aa1n02x5               g054(.a(\a[13] ), .b(\b[12] ), .o(new_n150));
  xorc02aa1n02x5               g055(.a(\a[13] ), .b(\b[12] ), .out0(new_n151));
  nanp02aa1n02x5               g056(.a(new_n148), .b(new_n151), .o1(new_n152));
  xorc02aa1n02x5               g057(.a(\a[14] ), .b(\b[13] ), .out0(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n153), .b(new_n152), .c(new_n150), .out0(\s[14] ));
  norp02aa1n02x5               g059(.a(\b[14] ), .b(\a[15] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  and002aa1n02x5               g062(.a(new_n153), .b(new_n151), .o(new_n158));
  oaoi03aa1n02x5               g063(.a(\a[14] ), .b(\b[13] ), .c(new_n150), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n157), .b(new_n159), .c(new_n148), .d(new_n158), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n157), .b(new_n159), .c(new_n148), .d(new_n158), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(\s[15] ));
  norp02aa1n02x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  nona22aa1n02x4               g070(.a(new_n160), .b(new_n165), .c(new_n155), .out0(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n165), .clkout(new_n167));
  oaoi13aa1n02x5               g072(.a(new_n167), .b(new_n160), .c(\a[15] ), .d(\b[14] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n166), .b(new_n168), .out0(\s[16] ));
  nano23aa1n02x4               g074(.a(new_n155), .b(new_n163), .c(new_n164), .d(new_n156), .out0(new_n170));
  nanp03aa1n02x5               g075(.a(new_n170), .b(new_n151), .c(new_n153), .o1(new_n171));
  norp02aa1n02x5               g076(.a(new_n171), .b(new_n144), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n173));
  oai012aa1n02x5               g078(.a(new_n164), .b(new_n163), .c(new_n155), .o1(new_n174));
  aobi12aa1n02x5               g079(.a(new_n174), .b(new_n170), .c(new_n159), .out0(new_n175));
  160nm_ficinv00aa1n08x5       g080(.clk(new_n175), .clkout(new_n176));
  oab012aa1n02x4               g081(.a(new_n176), .b(new_n147), .c(new_n171), .out0(new_n177));
  nanp02aa1n02x5               g082(.a(new_n173), .b(new_n177), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g084(.clk(\a[18] ), .clkout(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[17] ), .clkout(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(\b[16] ), .clkout(new_n182));
  oaoi03aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n178), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[17] ), .c(new_n180), .out0(\s[18] ));
  xroi22aa1d04x5               g089(.a(new_n181), .b(\b[16] ), .c(new_n180), .d(\b[17] ), .out0(new_n185));
  norp02aa1n02x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  aoi112aa1n02x5               g091(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n187));
  norp02aa1n02x5               g092(.a(new_n187), .b(new_n186), .o1(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(new_n188), .clkout(new_n189));
  norp02aa1n02x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n189), .c(new_n178), .d(new_n185), .o1(new_n193));
  aoi112aa1n02x5               g098(.a(new_n192), .b(new_n189), .c(new_n178), .d(new_n185), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  nona22aa1n02x4               g104(.a(new_n193), .b(new_n199), .c(new_n190), .out0(new_n200));
  160nm_ficinv00aa1n08x5       g105(.clk(new_n190), .clkout(new_n201));
  aobi12aa1n02x5               g106(.a(new_n199), .b(new_n193), .c(new_n201), .out0(new_n202));
  norb02aa1n02x5               g107(.a(new_n200), .b(new_n202), .out0(\s[20] ));
  nona23aa1n02x4               g108(.a(new_n198), .b(new_n191), .c(new_n190), .d(new_n197), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n204), .clkout(new_n205));
  nanp02aa1n02x5               g110(.a(new_n185), .b(new_n205), .o1(new_n206));
  oai012aa1n02x5               g111(.a(new_n198), .b(new_n197), .c(new_n190), .o1(new_n207));
  oai012aa1n02x5               g112(.a(new_n207), .b(new_n204), .c(new_n188), .o1(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n208), .clkout(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n206), .c(new_n173), .d(new_n177), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  xorc02aa1n02x5               g118(.a(\a[22] ), .b(\b[21] ), .out0(new_n214));
  aoi112aa1n02x5               g119(.a(new_n212), .b(new_n214), .c(new_n210), .d(new_n213), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n214), .b(new_n212), .c(new_n210), .d(new_n213), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(\s[22] ));
  nanp02aa1n02x5               g122(.a(new_n214), .b(new_n213), .o1(new_n218));
  nanb03aa1n02x5               g123(.a(new_n218), .b(new_n185), .c(new_n205), .out0(new_n219));
  oai112aa1n02x5               g124(.a(new_n192), .b(new_n199), .c(new_n187), .d(new_n186), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(\a[22] ), .clkout(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(\b[21] ), .clkout(new_n222));
  oao003aa1n02x5               g127(.a(new_n221), .b(new_n222), .c(new_n212), .carry(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n223), .clkout(new_n224));
  aoai13aa1n02x5               g129(.a(new_n224), .b(new_n218), .c(new_n220), .d(new_n207), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n225), .clkout(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n219), .c(new_n173), .d(new_n177), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g133(.a(\b[22] ), .b(\a[23] ), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[23] ), .b(\b[22] ), .out0(new_n230));
  xorc02aa1n02x5               g135(.a(\a[24] ), .b(\b[23] ), .out0(new_n231));
  aoi112aa1n02x5               g136(.a(new_n229), .b(new_n231), .c(new_n227), .d(new_n230), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n231), .b(new_n229), .c(new_n227), .d(new_n230), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(\s[24] ));
  and002aa1n02x5               g139(.a(new_n231), .b(new_n230), .o(new_n235));
  nona23aa1n02x4               g140(.a(new_n235), .b(new_n185), .c(new_n218), .d(new_n204), .out0(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(\a[24] ), .clkout(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(\b[23] ), .clkout(new_n238));
  oao003aa1n02x5               g143(.a(new_n237), .b(new_n238), .c(new_n229), .carry(new_n239));
  aoi012aa1n02x5               g144(.a(new_n239), .b(new_n225), .c(new_n235), .o1(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n236), .c(new_n173), .d(new_n177), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g147(.a(\b[24] ), .b(\a[25] ), .o1(new_n243));
  xorc02aa1n02x5               g148(.a(\a[25] ), .b(\b[24] ), .out0(new_n244));
  xorc02aa1n02x5               g149(.a(\a[26] ), .b(\b[25] ), .out0(new_n245));
  aoi112aa1n02x5               g150(.a(new_n243), .b(new_n245), .c(new_n241), .d(new_n244), .o1(new_n246));
  aoai13aa1n02x5               g151(.a(new_n245), .b(new_n243), .c(new_n241), .d(new_n244), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(\s[26] ));
  oao003aa1n02x5               g153(.a(new_n101), .b(new_n102), .c(new_n103), .carry(new_n249));
  nano23aa1n02x4               g154(.a(new_n105), .b(new_n107), .c(new_n108), .d(new_n106), .out0(new_n250));
  aobi12aa1n02x5               g155(.a(new_n110), .b(new_n250), .c(new_n249), .out0(new_n251));
  nano23aa1n02x4               g156(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n252));
  nanp03aa1n02x5               g157(.a(new_n252), .b(new_n117), .c(new_n118), .o1(new_n253));
  oai012aa1n02x5               g158(.a(new_n142), .b(new_n251), .c(new_n253), .o1(new_n254));
  oai012aa1n02x5               g159(.a(new_n175), .b(new_n147), .c(new_n171), .o1(new_n255));
  and002aa1n02x5               g160(.a(new_n245), .b(new_n244), .o(new_n256));
  nano22aa1n02x4               g161(.a(new_n219), .b(new_n235), .c(new_n256), .out0(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n255), .c(new_n254), .d(new_n172), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n256), .b(new_n239), .c(new_n225), .d(new_n235), .o1(new_n259));
  oai022aa1n02x5               g164(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n260));
  aob012aa1n02x5               g165(.a(new_n260), .b(\b[25] ), .c(\a[26] ), .out0(new_n261));
  xorc02aa1n02x5               g166(.a(\a[27] ), .b(\b[26] ), .out0(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(new_n262), .clkout(new_n263));
  aoi013aa1n02x4               g168(.a(new_n263), .b(new_n258), .c(new_n259), .d(new_n261), .o1(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n257), .clkout(new_n265));
  aoi012aa1n02x5               g170(.a(new_n265), .b(new_n173), .c(new_n177), .o1(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n218), .clkout(new_n267));
  aoai13aa1n02x5               g172(.a(new_n235), .b(new_n223), .c(new_n208), .d(new_n267), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n239), .clkout(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n256), .clkout(new_n270));
  aoai13aa1n02x5               g175(.a(new_n261), .b(new_n270), .c(new_n268), .d(new_n269), .o1(new_n271));
  norp03aa1n02x5               g176(.a(new_n271), .b(new_n266), .c(new_n262), .o1(new_n272));
  norp02aa1n02x5               g177(.a(new_n264), .b(new_n272), .o1(\s[27] ));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n274), .clkout(new_n275));
  oai012aa1n02x5               g180(.a(new_n262), .b(new_n271), .c(new_n266), .o1(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  aoi012aa1n02x5               g182(.a(new_n277), .b(new_n276), .c(new_n275), .o1(new_n278));
  nano22aa1n02x4               g183(.a(new_n264), .b(new_n275), .c(new_n277), .out0(new_n279));
  norp02aa1n02x5               g184(.a(new_n278), .b(new_n279), .o1(\s[28] ));
  norb02aa1n02x5               g185(.a(new_n262), .b(new_n277), .out0(new_n281));
  oai012aa1n02x5               g186(.a(new_n281), .b(new_n271), .c(new_n266), .o1(new_n282));
  oao003aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  aoi012aa1n02x5               g189(.a(new_n284), .b(new_n282), .c(new_n283), .o1(new_n285));
  160nm_ficinv00aa1n08x5       g190(.clk(new_n281), .clkout(new_n286));
  aoi013aa1n02x4               g191(.a(new_n286), .b(new_n258), .c(new_n259), .d(new_n261), .o1(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n283), .c(new_n284), .out0(new_n288));
  norp02aa1n02x5               g193(.a(new_n285), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n262), .b(new_n284), .c(new_n277), .out0(new_n291));
  oai012aa1n02x5               g196(.a(new_n291), .b(new_n271), .c(new_n266), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  160nm_ficinv00aa1n08x5       g200(.clk(new_n291), .clkout(new_n296));
  aoi013aa1n02x4               g201(.a(new_n296), .b(new_n258), .c(new_n259), .d(new_n261), .o1(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n293), .c(new_n294), .out0(new_n298));
  norp02aa1n02x5               g203(.a(new_n295), .b(new_n298), .o1(\s[30] ));
  norb02aa1n02x5               g204(.a(new_n291), .b(new_n294), .out0(new_n300));
  160nm_ficinv00aa1n08x5       g205(.clk(new_n300), .clkout(new_n301));
  aoi013aa1n02x4               g206(.a(new_n301), .b(new_n258), .c(new_n259), .d(new_n261), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n302), .b(new_n303), .c(new_n304), .out0(new_n305));
  oai012aa1n02x5               g210(.a(new_n300), .b(new_n271), .c(new_n266), .o1(new_n306));
  aoi012aa1n02x5               g211(.a(new_n304), .b(new_n306), .c(new_n303), .o1(new_n307));
  norp02aa1n02x5               g212(.a(new_n307), .b(new_n305), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n104), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n104), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g217(.a(\b[4] ), .b(\a[5] ), .o1(new_n313));
  oai012aa1n02x5               g218(.a(new_n313), .b(new_n111), .c(new_n122), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(new_n120), .out0(\s[6] ));
  oaoi03aa1n02x5               g220(.a(\a[5] ), .b(\b[4] ), .c(new_n251), .o1(new_n316));
  aobi12aa1n02x5               g221(.a(new_n123), .b(new_n316), .c(new_n117), .out0(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g223(.a(\a[7] ), .b(\b[6] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g225(.a(new_n126), .b(new_n141), .c(new_n142), .out0(\s[9] ));
endmodule


