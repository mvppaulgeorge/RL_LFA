// Benchmark "adder" written by ABC on Wed Jul 10 17:27:52 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n323, new_n325;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1n02x4               g006(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n102));
  norp02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  norp02aa1n02x5               g012(.a(new_n107), .b(new_n102), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  oai012aa1n02x5               g016(.a(new_n109), .b(new_n111), .c(new_n110), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  aoi012aa1n02x5               g022(.a(new_n113), .b(new_n115), .c(new_n114), .o1(new_n118));
  oai012aa1n02x5               g023(.a(new_n118), .b(new_n117), .c(new_n112), .o1(new_n119));
  aoi012aa1n02x5               g024(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n99), .b(new_n100), .c(new_n98), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n121), .b(new_n102), .c(new_n120), .o1(new_n122));
  160nm_fiao0012aa1n02p5x5     g027(.a(new_n122), .b(new_n119), .c(new_n108), .o(new_n123));
  nanp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  aoi012aa1n02x5               g029(.a(new_n97), .b(new_n123), .c(new_n124), .o1(new_n125));
  xnrb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nano23aa1n02x4               g036(.a(new_n97), .b(new_n130), .c(new_n131), .d(new_n124), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n122), .c(new_n119), .d(new_n108), .o1(new_n133));
  160nm_fiao0012aa1n02p5x5     g038(.a(new_n130), .b(new_n97), .c(new_n131), .o(new_n134));
  160nm_ficinv00aa1n08x5       g039(.clk(new_n134), .clkout(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n129), .b(new_n133), .c(new_n135), .out0(\s[11] ));
  nanp02aa1n02x5               g041(.a(new_n133), .b(new_n135), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n127), .b(new_n137), .c(new_n128), .o1(new_n138));
  xnrb03aa1n02x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nano23aa1n02x4               g046(.a(new_n127), .b(new_n140), .c(new_n141), .d(new_n128), .out0(new_n142));
  and002aa1n02x5               g047(.a(new_n142), .b(new_n132), .o(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n122), .c(new_n119), .d(new_n108), .o1(new_n144));
  oai012aa1n02x5               g049(.a(new_n141), .b(new_n140), .c(new_n127), .o1(new_n145));
  aobi12aa1n02x5               g050(.a(new_n145), .b(new_n142), .c(new_n134), .out0(new_n146));
  norp02aa1n02x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n144), .c(new_n146), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n147), .clkout(new_n151));
  aob012aa1n02x5               g056(.a(new_n149), .b(new_n144), .c(new_n146), .out0(new_n152));
  norp02aa1n02x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n152), .c(new_n151), .out0(\s[14] ));
  nano23aa1n02x4               g061(.a(new_n147), .b(new_n153), .c(new_n154), .d(new_n148), .out0(new_n157));
  160nm_ficinv00aa1n08x5       g062(.clk(new_n157), .clkout(new_n158));
  aoi012aa1n02x5               g063(.a(new_n153), .b(new_n147), .c(new_n154), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n158), .c(new_n144), .d(new_n146), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  norp02aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoi112aa1n02x5               g072(.a(new_n167), .b(new_n162), .c(new_n160), .d(new_n164), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n167), .b(new_n162), .c(new_n160), .d(new_n163), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(\s[16] ));
  nano23aa1n02x4               g075(.a(new_n162), .b(new_n165), .c(new_n166), .d(new_n163), .out0(new_n171));
  nanp02aa1n02x5               g076(.a(new_n171), .b(new_n157), .o1(new_n172));
  nano22aa1n02x4               g077(.a(new_n172), .b(new_n132), .c(new_n142), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n122), .c(new_n119), .d(new_n108), .o1(new_n174));
  160nm_ficinv00aa1n08x5       g079(.clk(new_n165), .clkout(new_n175));
  nanb03aa1n02x5               g080(.a(new_n159), .b(new_n167), .c(new_n164), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n162), .b(new_n166), .o1(new_n177));
  aoi112aa1n02x5               g082(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n141), .b(new_n140), .out0(new_n179));
  oai112aa1n02x5               g084(.a(new_n179), .b(new_n129), .c(new_n178), .d(new_n130), .o1(new_n180));
  aoi012aa1n02x5               g085(.a(new_n172), .b(new_n180), .c(new_n145), .o1(new_n181));
  nano32aa1n02x4               g086(.a(new_n181), .b(new_n177), .c(new_n176), .d(new_n175), .out0(new_n182));
  xorc02aa1n02x5               g087(.a(\a[17] ), .b(\b[16] ), .out0(new_n183));
  xnbna2aa1n03x5               g088(.a(new_n183), .b(new_n174), .c(new_n182), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[17] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\b[16] ), .clkout(new_n187));
  nanp02aa1n02x5               g092(.a(new_n174), .b(new_n182), .o1(new_n188));
  oaoi03aa1n02x5               g093(.a(new_n186), .b(new_n187), .c(new_n188), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d04x5               g095(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(new_n191), .clkout(new_n192));
  norp02aa1n02x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  aoi112aa1n02x5               g098(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n194));
  norp02aa1n02x5               g099(.a(new_n194), .b(new_n193), .o1(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n192), .c(new_n174), .d(new_n182), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  xorc02aa1n02x5               g104(.a(\a[19] ), .b(\b[18] ), .out0(new_n200));
  norp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  aoi112aa1n02x5               g108(.a(new_n199), .b(new_n203), .c(new_n196), .d(new_n200), .o1(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n199), .clkout(new_n205));
  nanp02aa1n02x5               g110(.a(new_n196), .b(new_n200), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n201), .b(new_n202), .out0(new_n207));
  aoi012aa1n02x5               g112(.a(new_n207), .b(new_n206), .c(new_n205), .o1(new_n208));
  norp02aa1n02x5               g113(.a(new_n208), .b(new_n204), .o1(\s[20] ));
  xnrc02aa1n02x5               g114(.a(\b[18] ), .b(\a[19] ), .out0(new_n210));
  norp02aa1n02x5               g115(.a(new_n210), .b(new_n207), .o1(new_n211));
  nanp02aa1n02x5               g116(.a(new_n191), .b(new_n211), .o1(new_n212));
  oaoi03aa1n02x5               g117(.a(\a[20] ), .b(\b[19] ), .c(new_n205), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  oai013aa1n02x4               g119(.a(new_n214), .b(new_n195), .c(new_n210), .d(new_n207), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n212), .c(new_n174), .d(new_n182), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  norp02aa1n02x5               g126(.a(\b[21] ), .b(\a[22] ), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(\b[21] ), .b(\a[22] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  aoi112aa1n02x5               g129(.a(new_n219), .b(new_n224), .c(new_n217), .d(new_n221), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(new_n219), .clkout(new_n226));
  nanp02aa1n02x5               g131(.a(new_n217), .b(new_n221), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n224), .clkout(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n227), .c(new_n226), .o1(new_n229));
  norp02aa1n02x5               g134(.a(new_n229), .b(new_n225), .o1(\s[22] ));
  nano23aa1n02x4               g135(.a(new_n219), .b(new_n222), .c(new_n223), .d(new_n220), .out0(new_n231));
  nanp03aa1n02x5               g136(.a(new_n191), .b(new_n211), .c(new_n231), .o1(new_n232));
  oaoi03aa1n02x5               g137(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n215), .c(new_n231), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n232), .c(new_n174), .d(new_n182), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  norp02aa1n02x5               g144(.a(\b[23] ), .b(\a[24] ), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n241), .b(new_n240), .out0(new_n242));
  aoi112aa1n02x5               g147(.a(new_n237), .b(new_n242), .c(new_n235), .d(new_n239), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n237), .clkout(new_n244));
  nanp02aa1n02x5               g149(.a(new_n235), .b(new_n239), .o1(new_n245));
  160nm_ficinv00aa1n08x5       g150(.clk(new_n242), .clkout(new_n246));
  aoi012aa1n02x5               g151(.a(new_n246), .b(new_n245), .c(new_n244), .o1(new_n247));
  norp02aa1n02x5               g152(.a(new_n247), .b(new_n243), .o1(\s[24] ));
  nano23aa1n02x4               g153(.a(new_n237), .b(new_n240), .c(new_n241), .d(new_n238), .out0(new_n249));
  nanb03aa1n02x5               g154(.a(new_n212), .b(new_n249), .c(new_n231), .out0(new_n250));
  nano32aa1n02x4               g155(.a(new_n246), .b(new_n239), .c(new_n224), .d(new_n221), .out0(new_n251));
  aoi112aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(new_n249), .b(new_n233), .o1(new_n253));
  nona22aa1n02x4               g158(.a(new_n253), .b(new_n252), .c(new_n240), .out0(new_n254));
  aoi012aa1n02x5               g159(.a(new_n254), .b(new_n215), .c(new_n251), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n250), .c(new_n174), .d(new_n182), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[25] ), .b(\b[24] ), .out0(new_n259));
  xorc02aa1n02x5               g164(.a(\a[26] ), .b(\b[25] ), .out0(new_n260));
  aoi112aa1n02x5               g165(.a(new_n258), .b(new_n260), .c(new_n256), .d(new_n259), .o1(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n258), .clkout(new_n262));
  nanp02aa1n02x5               g167(.a(new_n256), .b(new_n259), .o1(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n260), .clkout(new_n264));
  aoi012aa1n02x5               g169(.a(new_n264), .b(new_n263), .c(new_n262), .o1(new_n265));
  norp02aa1n02x5               g170(.a(new_n265), .b(new_n261), .o1(\s[26] ));
  and002aa1n02x5               g171(.a(new_n260), .b(new_n259), .o(new_n267));
  nano22aa1n02x4               g172(.a(new_n232), .b(new_n267), .c(new_n249), .out0(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n268), .clkout(new_n269));
  oai112aa1n02x5               g174(.a(new_n200), .b(new_n203), .c(new_n194), .d(new_n193), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(new_n249), .b(new_n231), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n271), .b(new_n270), .c(new_n214), .o1(new_n272));
  oaoi03aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .o1(new_n273));
  oaoi13aa1n02x5               g178(.a(new_n273), .b(new_n267), .c(new_n272), .d(new_n254), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n269), .c(new_n174), .d(new_n182), .o1(new_n275));
  xorb03aa1n02x5               g180(.a(new_n275), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  xorc02aa1n02x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xorc02aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .out0(new_n279));
  aoi112aa1n02x5               g184(.a(new_n277), .b(new_n279), .c(new_n275), .d(new_n278), .o1(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n277), .clkout(new_n281));
  aoai13aa1n02x5               g186(.a(new_n267), .b(new_n254), .c(new_n215), .d(new_n251), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n273), .clkout(new_n283));
  nanp02aa1n02x5               g188(.a(new_n282), .b(new_n283), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n278), .b(new_n284), .c(new_n188), .d(new_n268), .o1(new_n285));
  160nm_ficinv00aa1n08x5       g190(.clk(new_n279), .clkout(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n285), .c(new_n281), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n280), .o1(\s[28] ));
  and002aa1n02x5               g193(.a(new_n279), .b(new_n278), .o(new_n289));
  aoai13aa1n02x5               g194(.a(new_n289), .b(new_n284), .c(new_n188), .d(new_n268), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .carry(new_n291));
  xorc02aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .out0(new_n292));
  160nm_ficinv00aa1n08x5       g197(.clk(new_n292), .clkout(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n290), .c(new_n291), .o1(new_n294));
  160nm_ficinv00aa1n08x5       g199(.clk(new_n291), .clkout(new_n295));
  aoi112aa1n02x5               g200(.a(new_n292), .b(new_n295), .c(new_n275), .d(new_n289), .o1(new_n296));
  norp02aa1n02x5               g201(.a(new_n294), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g203(.a(new_n293), .b(new_n278), .c(new_n279), .out0(new_n299));
  aoai13aa1n02x5               g204(.a(new_n299), .b(new_n284), .c(new_n188), .d(new_n268), .o1(new_n300));
  oaoi03aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .o1(new_n301));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n301), .clkout(new_n302));
  xorc02aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .out0(new_n303));
  160nm_ficinv00aa1n08x5       g208(.clk(new_n303), .clkout(new_n304));
  aoi012aa1n02x5               g209(.a(new_n304), .b(new_n300), .c(new_n302), .o1(new_n305));
  aoi112aa1n02x5               g210(.a(new_n303), .b(new_n301), .c(new_n275), .d(new_n299), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n305), .b(new_n306), .o1(\s[30] ));
  nano32aa1n02x4               g212(.a(new_n304), .b(new_n292), .c(new_n279), .d(new_n278), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n309));
  160nm_ficinv00aa1n08x5       g214(.clk(new_n309), .clkout(new_n310));
  xorc02aa1n02x5               g215(.a(\a[31] ), .b(\b[30] ), .out0(new_n311));
  aoi112aa1n02x5               g216(.a(new_n311), .b(new_n310), .c(new_n275), .d(new_n308), .o1(new_n312));
  aoai13aa1n02x5               g217(.a(new_n308), .b(new_n284), .c(new_n188), .d(new_n268), .o1(new_n313));
  160nm_ficinv00aa1n08x5       g218(.clk(new_n311), .clkout(new_n314));
  aoi012aa1n02x5               g219(.a(new_n314), .b(new_n313), .c(new_n309), .o1(new_n315));
  norp02aa1n02x5               g220(.a(new_n315), .b(new_n312), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n112), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n112), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g225(.a(new_n105), .b(new_n119), .c(new_n106), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g227(.a(new_n120), .b(new_n107), .c(new_n119), .out0(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g229(.a(new_n100), .b(new_n323), .c(new_n101), .o1(new_n325));
  xnrb03aa1n02x5               g230(.a(new_n325), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g231(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

