// Benchmark "adder" written by ABC on Thu Jul 11 12:09:00 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n320,
    new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norb02aa1n02x5               g004(.a(new_n99), .b(new_n98), .out0(new_n100));
  xorc02aa1n02x5               g005(.a(\a[7] ), .b(\b[6] ), .out0(new_n101));
  norp02aa1n02x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  nanb03aa1n02x5               g011(.a(new_n106), .b(new_n100), .c(new_n101), .out0(new_n107));
  norp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  aoi012aa1n02x5               g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  oa0012aa1n02x5               g021(.a(new_n113), .b(new_n114), .c(new_n112), .o(new_n117));
  oab012aa1n02x4               g022(.a(new_n117), .b(new_n116), .c(new_n111), .out0(new_n118));
  aoi112aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n119));
  aoi112aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n120));
  oai112aa1n02x5               g025(.a(new_n101), .b(new_n100), .c(new_n102), .d(new_n120), .o1(new_n121));
  nona22aa1n02x4               g026(.a(new_n121), .b(new_n119), .c(new_n98), .out0(new_n122));
  oabi12aa1n02x5               g027(.a(new_n122), .b(new_n118), .c(new_n107), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanb02aa1n02x5               g031(.a(new_n125), .b(new_n126), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n97), .c(new_n123), .d(new_n124), .o1(new_n128));
  nano22aa1n02x4               g033(.a(new_n106), .b(new_n101), .c(new_n100), .out0(new_n129));
  oabi12aa1n02x5               g034(.a(new_n117), .b(new_n111), .c(new_n116), .out0(new_n130));
  norb02aa1n02x5               g035(.a(new_n124), .b(new_n97), .out0(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n122), .c(new_n130), .d(new_n129), .o1(new_n132));
  nona22aa1n02x4               g037(.a(new_n132), .b(new_n127), .c(new_n97), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(new_n128), .b(new_n133), .o1(\s[10] ));
  norp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n133), .c(new_n126), .out0(\s[11] ));
  aoi013aa1n02x4               g043(.a(new_n135), .b(new_n133), .c(new_n126), .d(new_n136), .o1(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  160nm_ficinv00aa1n08x5       g045(.clk(new_n140), .clkout(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n139), .b(new_n142), .c(new_n141), .out0(\s[12] ));
  norb02aa1n02x5               g048(.a(new_n142), .b(new_n140), .out0(new_n144));
  nano23aa1n02x4               g049(.a(new_n137), .b(new_n127), .c(new_n144), .d(new_n131), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n122), .c(new_n130), .d(new_n129), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n135), .b(new_n142), .o1(new_n147));
  nanb02aa1n02x5               g052(.a(new_n140), .b(new_n142), .out0(new_n148));
  oai022aa1n02x5               g053(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n149));
  nano23aa1n02x4               g054(.a(new_n148), .b(new_n137), .c(new_n149), .d(new_n126), .out0(new_n150));
  nano22aa1n02x4               g055(.a(new_n150), .b(new_n141), .c(new_n147), .out0(new_n151));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n146), .c(new_n151), .out0(\s[13] ));
  nanp02aa1n02x5               g060(.a(new_n146), .b(new_n151), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n152), .b(new_n156), .c(new_n153), .o1(new_n157));
  norp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  xnrc02aa1n02x5               g065(.a(new_n157), .b(new_n160), .out0(\s[14] ));
  norp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  nano23aa1n02x4               g069(.a(new_n152), .b(new_n158), .c(new_n159), .d(new_n153), .out0(new_n165));
  160nm_fiao0012aa1n02p5x5     g070(.a(new_n158), .b(new_n152), .c(new_n159), .o(new_n166));
  aoai13aa1n02x5               g071(.a(new_n164), .b(new_n166), .c(new_n156), .d(new_n165), .o1(new_n167));
  aoi112aa1n02x5               g072(.a(new_n164), .b(new_n166), .c(new_n156), .d(new_n165), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(\s[15] ));
  norp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  nona22aa1n02x4               g077(.a(new_n167), .b(new_n172), .c(new_n162), .out0(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n172), .clkout(new_n174));
  oaoi13aa1n02x5               g079(.a(new_n174), .b(new_n167), .c(\a[15] ), .d(\b[14] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n173), .b(new_n175), .out0(\s[16] ));
  norb02aa1n02x5               g081(.a(new_n136), .b(new_n135), .out0(new_n177));
  nano23aa1n02x4               g082(.a(new_n97), .b(new_n125), .c(new_n126), .d(new_n124), .out0(new_n178));
  nano23aa1n02x4               g083(.a(new_n162), .b(new_n170), .c(new_n171), .d(new_n163), .out0(new_n179));
  nanp02aa1n02x5               g084(.a(new_n179), .b(new_n165), .o1(new_n180));
  nano32aa1n02x4               g085(.a(new_n180), .b(new_n178), .c(new_n144), .d(new_n177), .out0(new_n181));
  aoai13aa1n02x5               g086(.a(new_n181), .b(new_n122), .c(new_n130), .d(new_n129), .o1(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(new_n126), .clkout(new_n183));
  norp02aa1n02x5               g088(.a(new_n125), .b(new_n97), .o1(new_n184));
  nona23aa1n02x4               g089(.a(new_n144), .b(new_n177), .c(new_n184), .d(new_n183), .out0(new_n185));
  nanp03aa1n02x5               g090(.a(new_n185), .b(new_n141), .c(new_n147), .o1(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(new_n180), .clkout(new_n187));
  aoi112aa1n02x5               g092(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n179), .b(new_n166), .o1(new_n189));
  nona22aa1n02x4               g094(.a(new_n189), .b(new_n188), .c(new_n170), .out0(new_n190));
  aoi012aa1n02x5               g095(.a(new_n190), .b(new_n186), .c(new_n187), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(new_n191), .b(new_n182), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g098(.clk(\a[18] ), .clkout(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(\a[17] ), .clkout(new_n195));
  160nm_ficinv00aa1n08x5       g100(.clk(\b[16] ), .clkout(new_n196));
  oaoi03aa1n02x5               g101(.a(new_n195), .b(new_n196), .c(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(new_n194), .out0(\s[18] ));
  xroi22aa1d04x5               g103(.a(new_n195), .b(\b[16] ), .c(new_n194), .d(\b[17] ), .out0(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n199), .clkout(new_n200));
  norp02aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  aoi013aa1n02x4               g107(.a(new_n201), .b(new_n202), .c(new_n195), .d(new_n196), .o1(new_n203));
  aoai13aa1n02x5               g108(.a(new_n203), .b(new_n200), .c(new_n191), .d(new_n182), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  norp02aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanp02aa1n02x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  aoi112aa1n02x5               g116(.a(new_n207), .b(new_n211), .c(new_n204), .d(new_n208), .o1(new_n212));
  aoai13aa1n02x5               g117(.a(new_n211), .b(new_n207), .c(new_n204), .d(new_n208), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(\s[20] ));
  nano23aa1n02x4               g119(.a(new_n207), .b(new_n209), .c(new_n210), .d(new_n208), .out0(new_n215));
  nanp02aa1n02x5               g120(.a(new_n199), .b(new_n215), .o1(new_n216));
  nona23aa1n02x4               g121(.a(new_n210), .b(new_n208), .c(new_n207), .d(new_n209), .out0(new_n217));
  oai012aa1n02x5               g122(.a(new_n210), .b(new_n209), .c(new_n207), .o1(new_n218));
  oai012aa1n02x5               g123(.a(new_n218), .b(new_n217), .c(new_n203), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n219), .clkout(new_n220));
  aoai13aa1n02x5               g125(.a(new_n220), .b(new_n216), .c(new_n191), .d(new_n182), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  aoi112aa1n02x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n225), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g133(.clk(\a[21] ), .clkout(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\a[22] ), .clkout(new_n230));
  xroi22aa1d04x5               g135(.a(new_n229), .b(\b[20] ), .c(new_n230), .d(\b[21] ), .out0(new_n231));
  nanp03aa1n02x5               g136(.a(new_n231), .b(new_n199), .c(new_n215), .o1(new_n232));
  nona22aa1n02x4               g137(.a(new_n202), .b(\b[16] ), .c(\a[17] ), .out0(new_n233));
  oaib12aa1n02x5               g138(.a(new_n233), .b(\b[17] ), .c(new_n194), .out0(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n218), .clkout(new_n235));
  aoai13aa1n02x5               g140(.a(new_n231), .b(new_n235), .c(new_n215), .d(new_n234), .o1(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(\b[21] ), .clkout(new_n237));
  oao003aa1n02x5               g142(.a(new_n230), .b(new_n237), .c(new_n223), .carry(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  nanp02aa1n02x5               g144(.a(new_n236), .b(new_n239), .o1(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n232), .c(new_n191), .d(new_n182), .o1(new_n242));
  xorb03aa1n02x5               g147(.a(new_n242), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  xorc02aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  xorc02aa1n02x5               g150(.a(\a[24] ), .b(\b[23] ), .out0(new_n246));
  aoi112aa1n02x5               g151(.a(new_n244), .b(new_n246), .c(new_n242), .d(new_n245), .o1(new_n247));
  aoai13aa1n02x5               g152(.a(new_n246), .b(new_n244), .c(new_n242), .d(new_n245), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n248), .b(new_n247), .out0(\s[24] ));
  and002aa1n02x5               g154(.a(new_n246), .b(new_n245), .o(new_n250));
  nanb03aa1n02x5               g155(.a(new_n216), .b(new_n250), .c(new_n231), .out0(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n250), .clkout(new_n252));
  nanp02aa1n02x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  oai022aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(new_n254), .b(new_n253), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n252), .c(new_n236), .d(new_n239), .o1(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n256), .clkout(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n251), .c(new_n191), .d(new_n182), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  xorc02aa1n02x5               g166(.a(\a[26] ), .b(\b[25] ), .out0(new_n262));
  aoi112aa1n02x5               g167(.a(new_n260), .b(new_n262), .c(new_n258), .d(new_n261), .o1(new_n263));
  aoai13aa1n02x5               g168(.a(new_n262), .b(new_n260), .c(new_n258), .d(new_n261), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n264), .b(new_n263), .out0(\s[26] ));
  oabi12aa1n02x5               g170(.a(new_n190), .b(new_n151), .c(new_n180), .out0(new_n266));
  and002aa1n02x5               g171(.a(new_n262), .b(new_n261), .o(new_n267));
  nano22aa1n02x4               g172(.a(new_n232), .b(new_n250), .c(new_n267), .out0(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n266), .c(new_n123), .d(new_n181), .o1(new_n269));
  oai022aa1n02x5               g174(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n270));
  aob012aa1n02x5               g175(.a(new_n270), .b(\b[25] ), .c(\a[26] ), .out0(new_n271));
  aobi12aa1n02x5               g176(.a(new_n271), .b(new_n256), .c(new_n267), .out0(new_n272));
  xorc02aa1n02x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n272), .c(new_n269), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n275), .clkout(new_n276));
  aobi12aa1n02x5               g181(.a(new_n273), .b(new_n272), .c(new_n269), .out0(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[27] ), .b(\a[28] ), .out0(new_n278));
  nano22aa1n02x4               g183(.a(new_n277), .b(new_n276), .c(new_n278), .out0(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n268), .clkout(new_n280));
  aoi012aa1n02x5               g185(.a(new_n280), .b(new_n191), .c(new_n182), .o1(new_n281));
  aoai13aa1n02x5               g186(.a(new_n250), .b(new_n238), .c(new_n219), .d(new_n231), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n267), .clkout(new_n283));
  aoai13aa1n02x5               g188(.a(new_n271), .b(new_n283), .c(new_n282), .d(new_n255), .o1(new_n284));
  oai012aa1n02x5               g189(.a(new_n273), .b(new_n284), .c(new_n281), .o1(new_n285));
  aoi012aa1n02x5               g190(.a(new_n278), .b(new_n285), .c(new_n276), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n286), .b(new_n279), .o1(\s[28] ));
  norb02aa1n02x5               g192(.a(new_n273), .b(new_n278), .out0(new_n288));
  oai012aa1n02x5               g193(.a(new_n288), .b(new_n284), .c(new_n281), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  aoi012aa1n02x5               g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x5               g197(.a(new_n288), .b(new_n272), .c(new_n269), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n02x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n273), .b(new_n291), .c(new_n278), .out0(new_n297));
  oai012aa1n02x5               g202(.a(new_n297), .b(new_n284), .c(new_n281), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n02x5               g206(.a(new_n297), .b(new_n272), .c(new_n269), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n02x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  norb02aa1n02x5               g209(.a(new_n297), .b(new_n300), .out0(new_n305));
  aobi12aa1n02x5               g210(.a(new_n305), .b(new_n272), .c(new_n269), .out0(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nano22aa1n02x4               g213(.a(new_n306), .b(new_n307), .c(new_n308), .out0(new_n309));
  oai012aa1n02x5               g214(.a(new_n305), .b(new_n284), .c(new_n281), .o1(new_n310));
  aoi012aa1n02x5               g215(.a(new_n308), .b(new_n310), .c(new_n307), .o1(new_n311));
  norp02aa1n02x5               g216(.a(new_n311), .b(new_n309), .o1(\s[31] ));
  xnrb03aa1n02x5               g217(.a(new_n111), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n130), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g221(.a(\a[5] ), .b(\b[4] ), .c(new_n118), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g223(.a(new_n101), .b(new_n102), .c(new_n317), .d(new_n103), .o1(new_n319));
  aoi112aa1n02x5               g224(.a(new_n101), .b(new_n102), .c(new_n317), .d(new_n103), .o1(new_n320));
  norb02aa1n02x5               g225(.a(new_n319), .b(new_n320), .out0(\s[7] ));
  orn002aa1n02x5               g226(.a(\a[7] ), .b(\b[6] ), .o(new_n322));
  xnbna2aa1n03x5               g227(.a(new_n100), .b(new_n319), .c(new_n322), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


