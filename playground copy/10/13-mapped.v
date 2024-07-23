// Benchmark "adder" written by ABC on Thu Jul 11 11:36:49 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n321, new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nanp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  norb02aa1n02x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  orn002aa1n02x5               g009(.a(\a[7] ), .b(\b[6] ), .o(new_n105));
  nanp02aa1n02x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  norp02aa1n02x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nona23aa1n02x4               g015(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n111));
  nano32aa1n02x4               g016(.a(new_n111), .b(new_n104), .c(new_n105), .d(new_n106), .out0(new_n112));
  norp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  nanp02aa1n02x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[0] ), .b(\a[1] ), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[1] ), .b(\a[2] ), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n118), .b(new_n120), .c(new_n119), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n114), .b(new_n115), .c(new_n113), .o1(new_n122));
  oai012aa1n02x5               g027(.a(new_n122), .b(new_n117), .c(new_n121), .o1(new_n123));
  aoi112aa1n02x5               g028(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n124));
  xorc02aa1n02x5               g029(.a(\a[7] ), .b(\b[6] ), .out0(new_n125));
  aoi112aa1n02x5               g030(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n126));
  oai112aa1n02x5               g031(.a(new_n125), .b(new_n104), .c(new_n107), .d(new_n126), .o1(new_n127));
  nona22aa1n02x4               g032(.a(new_n127), .b(new_n124), .c(new_n102), .out0(new_n128));
  aoi112aa1n02x5               g033(.a(new_n128), .b(new_n101), .c(new_n123), .d(new_n112), .o1(new_n129));
  nona22aa1n02x4               g034(.a(new_n100), .b(new_n129), .c(new_n99), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n99), .b(new_n129), .c(\b[8] ), .d(\a[9] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(new_n130), .b(new_n131), .o1(\s[10] ));
  norp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n131), .c(new_n98), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n133), .clkout(new_n137));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n98), .clkout(new_n138));
  nona22aa1n02x4               g043(.a(new_n131), .b(new_n135), .c(new_n138), .out0(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n137), .out0(\s[12] ));
  norb02aa1n02x5               g048(.a(new_n134), .b(new_n133), .out0(new_n144));
  nano23aa1n02x4               g049(.a(new_n101), .b(new_n97), .c(new_n98), .d(new_n100), .out0(new_n145));
  and003aa1n02x5               g050(.a(new_n145), .b(new_n142), .c(new_n144), .o(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n128), .c(new_n123), .d(new_n112), .o1(new_n147));
  160nm_ficinv00aa1n08x5       g052(.clk(new_n140), .clkout(new_n148));
  nanp02aa1n02x5               g053(.a(new_n133), .b(new_n141), .o1(new_n149));
  nanb02aa1n02x5               g054(.a(new_n140), .b(new_n141), .out0(new_n150));
  oai022aa1n02x5               g055(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n151));
  nano23aa1n02x4               g056(.a(new_n150), .b(new_n135), .c(new_n151), .d(new_n98), .out0(new_n152));
  nano22aa1n02x4               g057(.a(new_n152), .b(new_n148), .c(new_n149), .out0(new_n153));
  norp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n147), .c(new_n153), .out0(\s[13] ));
  nanp02aa1n02x5               g062(.a(new_n147), .b(new_n153), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n154), .b(new_n158), .c(new_n155), .o1(new_n159));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  xnrc02aa1n02x5               g067(.a(new_n159), .b(new_n162), .out0(\s[14] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  nano23aa1n02x4               g071(.a(new_n154), .b(new_n160), .c(new_n161), .d(new_n155), .out0(new_n167));
  160nm_fiao0012aa1n02p5x5     g072(.a(new_n160), .b(new_n154), .c(new_n161), .o(new_n168));
  aoai13aa1n02x5               g073(.a(new_n166), .b(new_n168), .c(new_n158), .d(new_n167), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n166), .b(new_n168), .c(new_n158), .d(new_n167), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(\s[15] ));
  norp02aa1n02x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  nona22aa1n02x4               g079(.a(new_n169), .b(new_n174), .c(new_n164), .out0(new_n175));
  160nm_ficinv00aa1n08x5       g080(.clk(new_n174), .clkout(new_n176));
  oaoi13aa1n02x5               g081(.a(new_n176), .b(new_n169), .c(\a[15] ), .d(\b[14] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n175), .b(new_n177), .out0(\s[16] ));
  nano23aa1n02x4               g083(.a(new_n164), .b(new_n172), .c(new_n173), .d(new_n165), .out0(new_n179));
  nanp02aa1n02x5               g084(.a(new_n179), .b(new_n167), .o1(new_n180));
  nano32aa1n02x4               g085(.a(new_n180), .b(new_n145), .c(new_n142), .d(new_n144), .out0(new_n181));
  aoai13aa1n02x5               g086(.a(new_n181), .b(new_n128), .c(new_n123), .d(new_n112), .o1(new_n182));
  norp02aa1n02x5               g087(.a(new_n101), .b(new_n97), .o1(new_n183));
  nona23aa1n02x4               g088(.a(new_n142), .b(new_n144), .c(new_n183), .d(new_n138), .out0(new_n184));
  nanp03aa1n02x5               g089(.a(new_n184), .b(new_n148), .c(new_n149), .o1(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(new_n180), .clkout(new_n186));
  aoi112aa1n02x5               g091(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(new_n179), .b(new_n168), .o1(new_n188));
  nona22aa1n02x4               g093(.a(new_n188), .b(new_n187), .c(new_n172), .out0(new_n189));
  aoi012aa1n02x5               g094(.a(new_n189), .b(new_n185), .c(new_n186), .o1(new_n190));
  xorc02aa1n02x5               g095(.a(\a[17] ), .b(\b[16] ), .out0(new_n191));
  xnbna2aa1n03x5               g096(.a(new_n191), .b(new_n190), .c(new_n182), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g097(.clk(\a[17] ), .clkout(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(\b[16] ), .clkout(new_n194));
  nanp02aa1n02x5               g099(.a(new_n194), .b(new_n193), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(new_n123), .b(new_n112), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n128), .b(new_n196), .out0(new_n197));
  oabi12aa1n02x5               g102(.a(new_n189), .b(new_n153), .c(new_n180), .out0(new_n198));
  aoai13aa1n02x5               g103(.a(new_n191), .b(new_n198), .c(new_n197), .d(new_n181), .o1(new_n199));
  norp02aa1n02x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n199), .c(new_n195), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g108(.clk(\a[18] ), .clkout(new_n204));
  xroi22aa1d04x5               g109(.a(new_n193), .b(\b[16] ), .c(new_n204), .d(\b[17] ), .out0(new_n205));
  160nm_ficinv00aa1n08x5       g110(.clk(new_n205), .clkout(new_n206));
  aoi013aa1n02x4               g111(.a(new_n200), .b(new_n201), .c(new_n193), .d(new_n194), .o1(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n206), .c(new_n190), .d(new_n182), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nanp02aa1n02x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  norp02aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  aoi112aa1n02x5               g120(.a(new_n211), .b(new_n215), .c(new_n208), .d(new_n212), .o1(new_n216));
  aoai13aa1n02x5               g121(.a(new_n215), .b(new_n211), .c(new_n208), .d(new_n212), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(\s[20] ));
  nano23aa1n02x4               g123(.a(new_n211), .b(new_n213), .c(new_n214), .d(new_n212), .out0(new_n219));
  nanp03aa1n02x5               g124(.a(new_n219), .b(new_n191), .c(new_n202), .o1(new_n220));
  nona23aa1n02x4               g125(.a(new_n214), .b(new_n212), .c(new_n211), .d(new_n213), .out0(new_n221));
  oai012aa1n02x5               g126(.a(new_n214), .b(new_n213), .c(new_n211), .o1(new_n222));
  oai012aa1n02x5               g127(.a(new_n222), .b(new_n221), .c(new_n207), .o1(new_n223));
  160nm_ficinv00aa1n08x5       g128(.clk(new_n223), .clkout(new_n224));
  aoai13aa1n02x5               g129(.a(new_n224), .b(new_n220), .c(new_n190), .d(new_n182), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  xorc02aa1n02x5               g132(.a(\a[21] ), .b(\b[20] ), .out0(new_n228));
  xorc02aa1n02x5               g133(.a(\a[22] ), .b(\b[21] ), .out0(new_n229));
  aoi112aa1n02x5               g134(.a(new_n227), .b(new_n229), .c(new_n225), .d(new_n228), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n229), .b(new_n227), .c(new_n225), .d(new_n228), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g137(.clk(\a[21] ), .clkout(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(\a[22] ), .clkout(new_n234));
  xroi22aa1d04x5               g139(.a(new_n233), .b(\b[20] ), .c(new_n234), .d(\b[21] ), .out0(new_n235));
  nanp03aa1n02x5               g140(.a(new_n235), .b(new_n205), .c(new_n219), .o1(new_n236));
  oaoi03aa1n02x5               g141(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n222), .clkout(new_n238));
  aoai13aa1n02x5               g143(.a(new_n235), .b(new_n238), .c(new_n219), .d(new_n237), .o1(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(\b[21] ), .clkout(new_n240));
  oao003aa1n02x5               g145(.a(new_n234), .b(new_n240), .c(new_n227), .carry(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n241), .clkout(new_n242));
  nanp02aa1n02x5               g147(.a(new_n239), .b(new_n242), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n236), .c(new_n190), .d(new_n182), .o1(new_n245));
  xorb03aa1n02x5               g150(.a(new_n245), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[23] ), .b(\b[22] ), .out0(new_n248));
  xorc02aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  aoi112aa1n02x5               g154(.a(new_n247), .b(new_n249), .c(new_n245), .d(new_n248), .o1(new_n250));
  aoai13aa1n02x5               g155(.a(new_n249), .b(new_n247), .c(new_n245), .d(new_n248), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(\s[24] ));
  and002aa1n02x5               g157(.a(new_n249), .b(new_n248), .o(new_n253));
  nanb03aa1n02x5               g158(.a(new_n220), .b(new_n253), .c(new_n235), .out0(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n253), .clkout(new_n255));
  nanp02aa1n02x5               g160(.a(\b[23] ), .b(\a[24] ), .o1(new_n256));
  oai022aa1n02x5               g161(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n257), .b(new_n256), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n255), .c(new_n239), .d(new_n242), .o1(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n259), .clkout(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n254), .c(new_n190), .d(new_n182), .o1(new_n261));
  xorb03aa1n02x5               g166(.a(new_n261), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  xorc02aa1n02x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  xorc02aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .out0(new_n265));
  aoi112aa1n02x5               g170(.a(new_n263), .b(new_n265), .c(new_n261), .d(new_n264), .o1(new_n266));
  aoai13aa1n02x5               g171(.a(new_n265), .b(new_n263), .c(new_n261), .d(new_n264), .o1(new_n267));
  norb02aa1n02x5               g172(.a(new_n267), .b(new_n266), .out0(\s[26] ));
  and002aa1n02x5               g173(.a(new_n265), .b(new_n264), .o(new_n269));
  nano22aa1n02x4               g174(.a(new_n236), .b(new_n253), .c(new_n269), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n270), .b(new_n198), .c(new_n197), .d(new_n181), .o1(new_n271));
  oai022aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n272));
  aob012aa1n02x5               g177(.a(new_n272), .b(\b[25] ), .c(\a[26] ), .out0(new_n273));
  aobi12aa1n02x5               g178(.a(new_n273), .b(new_n259), .c(new_n269), .out0(new_n274));
  xorc02aa1n02x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n274), .c(new_n271), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n277), .clkout(new_n278));
  aobi12aa1n02x5               g183(.a(new_n275), .b(new_n274), .c(new_n271), .out0(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n279), .b(new_n278), .c(new_n280), .out0(new_n281));
  160nm_ficinv00aa1n08x5       g186(.clk(new_n270), .clkout(new_n282));
  aoi012aa1n02x5               g187(.a(new_n282), .b(new_n190), .c(new_n182), .o1(new_n283));
  aoai13aa1n02x5               g188(.a(new_n253), .b(new_n241), .c(new_n223), .d(new_n235), .o1(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n269), .clkout(new_n285));
  aoai13aa1n02x5               g190(.a(new_n273), .b(new_n285), .c(new_n284), .d(new_n258), .o1(new_n286));
  oai012aa1n02x5               g191(.a(new_n275), .b(new_n286), .c(new_n283), .o1(new_n287));
  aoi012aa1n02x5               g192(.a(new_n280), .b(new_n287), .c(new_n278), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n288), .b(new_n281), .o1(\s[28] ));
  norb02aa1n02x5               g194(.a(new_n275), .b(new_n280), .out0(new_n290));
  oai012aa1n02x5               g195(.a(new_n290), .b(new_n286), .c(new_n283), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  aobi12aa1n02x5               g199(.a(new_n290), .b(new_n274), .c(new_n271), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n292), .c(new_n293), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n294), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n119), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n275), .b(new_n293), .c(new_n280), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n299), .b(new_n286), .c(new_n283), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n300), .c(new_n301), .o1(new_n303));
  aobi12aa1n02x5               g208(.a(new_n299), .b(new_n274), .c(new_n271), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n304), .b(new_n301), .c(new_n302), .out0(new_n305));
  norp02aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[30] ));
  norb02aa1n02x5               g211(.a(new_n299), .b(new_n302), .out0(new_n307));
  aobi12aa1n02x5               g212(.a(new_n307), .b(new_n274), .c(new_n271), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nano22aa1n02x4               g215(.a(new_n308), .b(new_n309), .c(new_n310), .out0(new_n311));
  oai012aa1n02x5               g216(.a(new_n307), .b(new_n286), .c(new_n283), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n310), .b(new_n312), .c(new_n309), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnrb03aa1n02x5               g219(.a(new_n121), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n121), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n123), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  160nm_fiao0012aa1n02p5x5     g223(.a(new_n109), .b(new_n123), .c(new_n110), .o(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g225(.a(new_n125), .b(new_n107), .c(new_n319), .d(new_n108), .o1(new_n321));
  aoi112aa1n02x5               g226(.a(new_n125), .b(new_n107), .c(new_n319), .d(new_n108), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n321), .b(new_n322), .out0(\s[7] ));
  xnbna2aa1n03x5               g228(.a(new_n104), .b(new_n321), .c(new_n105), .out0(\s[8] ));
  xorb03aa1n02x5               g229(.a(new_n197), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


