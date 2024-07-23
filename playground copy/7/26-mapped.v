// Benchmark "adder" written by ABC on Thu Jul 11 11:25:17 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n304, new_n307, new_n309, new_n310, new_n312;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\a[2] ), .clkout(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\b[1] ), .clkout(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oaoi03aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[6] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\b[5] ), .clkout(new_n119));
  norp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n122));
  oai012aa1n02x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .o1(new_n123));
  160nm_ficinv00aa1n08x5       g028(.clk(new_n123), .clkout(new_n124));
  aob012aa1n02x5               g029(.a(new_n124), .b(new_n109), .c(new_n117), .out0(new_n125));
  xnrc02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .out0(new_n126));
  aoib12aa1n02x5               g031(.a(new_n98), .b(new_n125), .c(new_n126), .out0(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  norp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  xnrc02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .out0(new_n132));
  norp02aa1n02x5               g037(.a(new_n126), .b(new_n132), .o1(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n134));
  160nm_ficinv00aa1n08x5       g039(.clk(\b[9] ), .clkout(new_n135));
  oaoi03aa1n02x5               g040(.a(new_n97), .b(new_n135), .c(new_n98), .o1(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n131), .b(new_n134), .c(new_n136), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n129), .clkout(new_n138));
  160nm_ficinv00aa1n08x5       g043(.clk(new_n136), .clkout(new_n139));
  aoai13aa1n02x5               g044(.a(new_n131), .b(new_n139), .c(new_n125), .d(new_n133), .o1(new_n140));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n140), .c(new_n138), .out0(\s[12] ));
  nona23aa1n02x4               g049(.a(new_n142), .b(new_n130), .c(new_n129), .d(new_n141), .out0(new_n145));
  norp03aa1n02x5               g050(.a(new_n145), .b(new_n126), .c(new_n132), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n147));
  oai012aa1n02x5               g052(.a(new_n142), .b(new_n141), .c(new_n129), .o1(new_n148));
  oai012aa1n02x5               g053(.a(new_n148), .b(new_n145), .c(new_n136), .o1(new_n149));
  160nm_ficinv00aa1n08x5       g054(.clk(new_n149), .clkout(new_n150));
  nanp02aa1n02x5               g055(.a(new_n147), .b(new_n150), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n151), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nano23aa1n02x4               g063(.a(new_n153), .b(new_n157), .c(new_n158), .d(new_n154), .out0(new_n159));
  160nm_ficinv00aa1n08x5       g064(.clk(new_n159), .clkout(new_n160));
  aoi012aa1n02x5               g065(.a(new_n157), .b(new_n153), .c(new_n158), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n160), .c(new_n147), .d(new_n150), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  xorc02aa1n02x5               g069(.a(\a[15] ), .b(\b[14] ), .out0(new_n165));
  xorc02aa1n02x5               g070(.a(\a[16] ), .b(\b[15] ), .out0(new_n166));
  aoi112aa1n02x5               g071(.a(new_n166), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n166), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(\s[16] ));
  nano23aa1n02x4               g074(.a(new_n129), .b(new_n141), .c(new_n142), .d(new_n130), .out0(new_n170));
  nanp03aa1n02x5               g075(.a(new_n159), .b(new_n165), .c(new_n166), .o1(new_n171));
  nano22aa1n02x4               g076(.a(new_n171), .b(new_n133), .c(new_n170), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n173));
  nanb03aa1n02x5               g078(.a(new_n161), .b(new_n166), .c(new_n165), .out0(new_n174));
  orn002aa1n02x5               g079(.a(\a[15] ), .b(\b[14] ), .o(new_n175));
  oaoi03aa1n02x5               g080(.a(\a[16] ), .b(\b[15] ), .c(new_n175), .o1(new_n176));
  nanb02aa1n02x5               g081(.a(new_n176), .b(new_n174), .out0(new_n177));
  aoib12aa1n02x5               g082(.a(new_n177), .b(new_n149), .c(new_n171), .out0(new_n178));
  nanp02aa1n02x5               g083(.a(new_n173), .b(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g085(.clk(\a[18] ), .clkout(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(\a[17] ), .clkout(new_n182));
  160nm_ficinv00aa1n08x5       g087(.clk(\b[16] ), .clkout(new_n183));
  oaoi03aa1n02x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  xroi22aa1d04x5               g090(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nona22aa1n02x4               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(new_n188));
  oaib12aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n181), .out0(new_n189));
  norp02aa1n02x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n189), .c(new_n179), .d(new_n186), .o1(new_n193));
  aoi112aa1n02x5               g098(.a(new_n192), .b(new_n189), .c(new_n179), .d(new_n186), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  nona22aa1n02x4               g104(.a(new_n193), .b(new_n199), .c(new_n190), .out0(new_n200));
  orn002aa1n02x5               g105(.a(\a[19] ), .b(\b[18] ), .o(new_n201));
  aobi12aa1n02x5               g106(.a(new_n199), .b(new_n193), .c(new_n201), .out0(new_n202));
  norb02aa1n02x5               g107(.a(new_n200), .b(new_n202), .out0(\s[20] ));
  nano23aa1n02x4               g108(.a(new_n190), .b(new_n197), .c(new_n198), .d(new_n191), .out0(new_n204));
  nanp02aa1n02x5               g109(.a(new_n186), .b(new_n204), .o1(new_n205));
  norp02aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  aoi013aa1n02x4               g111(.a(new_n206), .b(new_n187), .c(new_n182), .d(new_n183), .o1(new_n207));
  nona23aa1n02x4               g112(.a(new_n198), .b(new_n191), .c(new_n190), .d(new_n197), .out0(new_n208));
  oaoi03aa1n02x5               g113(.a(\a[20] ), .b(\b[19] ), .c(new_n201), .o1(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  oai012aa1n02x5               g115(.a(new_n210), .b(new_n208), .c(new_n207), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(new_n211), .clkout(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n205), .c(new_n173), .d(new_n178), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  xorc02aa1n02x5               g120(.a(\a[21] ), .b(\b[20] ), .out0(new_n216));
  xorc02aa1n02x5               g121(.a(\a[22] ), .b(\b[21] ), .out0(new_n217));
  aoi112aa1n02x5               g122(.a(new_n215), .b(new_n217), .c(new_n213), .d(new_n216), .o1(new_n218));
  aoai13aa1n02x5               g123(.a(new_n217), .b(new_n215), .c(new_n213), .d(new_n216), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g125(.clk(\a[21] ), .clkout(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(\a[22] ), .clkout(new_n222));
  xroi22aa1d04x5               g127(.a(new_n221), .b(\b[20] ), .c(new_n222), .d(\b[21] ), .out0(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n209), .c(new_n204), .d(new_n189), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\b[21] ), .clkout(new_n225));
  oaoi03aa1n02x5               g130(.a(new_n222), .b(new_n225), .c(new_n215), .o1(new_n226));
  nanp02aa1n02x5               g131(.a(new_n224), .b(new_n226), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n227), .clkout(new_n228));
  nanp03aa1n02x5               g133(.a(new_n223), .b(new_n186), .c(new_n204), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n228), .b(new_n229), .c(new_n173), .d(new_n178), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  and002aa1n02x5               g142(.a(new_n234), .b(new_n233), .o(new_n238));
  nanb03aa1n02x5               g143(.a(new_n205), .b(new_n238), .c(new_n223), .out0(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n238), .clkout(new_n240));
  oai022aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n241));
  aob012aa1n02x5               g146(.a(new_n241), .b(\b[23] ), .c(\a[24] ), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n240), .c(new_n224), .d(new_n226), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n239), .c(new_n173), .d(new_n178), .o1(new_n245));
  xorb03aa1n02x5               g150(.a(new_n245), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g151(.a(\b[24] ), .b(\a[25] ), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  norp02aa1n02x5               g153(.a(\b[25] ), .b(\a[26] ), .o1(new_n249));
  nanp02aa1n02x5               g154(.a(\b[25] ), .b(\a[26] ), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n249), .out0(new_n251));
  aoi112aa1n02x5               g156(.a(new_n247), .b(new_n251), .c(new_n245), .d(new_n248), .o1(new_n252));
  aoai13aa1n02x5               g157(.a(new_n251), .b(new_n247), .c(new_n245), .d(new_n248), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n253), .b(new_n252), .out0(\s[26] ));
  oabi12aa1n02x5               g159(.a(new_n177), .b(new_n150), .c(new_n171), .out0(new_n255));
  and002aa1n02x5               g160(.a(new_n248), .b(new_n251), .o(new_n256));
  nano22aa1n02x4               g161(.a(new_n229), .b(new_n238), .c(new_n256), .out0(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n255), .c(new_n125), .d(new_n172), .o1(new_n258));
  oai012aa1n02x5               g163(.a(new_n250), .b(new_n249), .c(new_n247), .o1(new_n259));
  aobi12aa1n02x5               g164(.a(new_n259), .b(new_n243), .c(new_n256), .out0(new_n260));
  norp02aa1n02x5               g165(.a(\b[26] ), .b(\a[27] ), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(\b[26] ), .b(\a[27] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n260), .c(new_n258), .out0(\s[27] ));
  norp02aa1n02x5               g169(.a(\b[27] ), .b(\a[28] ), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(\b[27] ), .b(\a[28] ), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n266), .b(new_n265), .out0(new_n267));
  aobi12aa1n02x5               g172(.a(new_n257), .b(new_n173), .c(new_n178), .out0(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n226), .clkout(new_n269));
  aoai13aa1n02x5               g174(.a(new_n238), .b(new_n269), .c(new_n211), .d(new_n223), .o1(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n256), .clkout(new_n271));
  aoai13aa1n02x5               g176(.a(new_n259), .b(new_n271), .c(new_n270), .d(new_n242), .o1(new_n272));
  norp03aa1n02x5               g177(.a(new_n272), .b(new_n268), .c(new_n261), .o1(new_n273));
  nano22aa1n02x4               g178(.a(new_n273), .b(new_n262), .c(new_n267), .out0(new_n274));
  oai112aa1n02x5               g179(.a(new_n260), .b(new_n258), .c(\b[26] ), .d(\a[27] ), .o1(new_n275));
  aoi012aa1n02x5               g180(.a(new_n267), .b(new_n275), .c(new_n262), .o1(new_n276));
  norp02aa1n02x5               g181(.a(new_n276), .b(new_n274), .o1(\s[28] ));
  nano23aa1n02x4               g182(.a(new_n261), .b(new_n265), .c(new_n266), .d(new_n262), .out0(new_n278));
  oai012aa1n02x5               g183(.a(new_n278), .b(new_n272), .c(new_n268), .o1(new_n279));
  aoi012aa1n02x5               g184(.a(new_n265), .b(new_n261), .c(new_n266), .o1(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[28] ), .b(\a[29] ), .out0(new_n281));
  aoi012aa1n02x5               g186(.a(new_n281), .b(new_n279), .c(new_n280), .o1(new_n282));
  aobi12aa1n02x5               g187(.a(new_n278), .b(new_n260), .c(new_n258), .out0(new_n283));
  nano22aa1n02x4               g188(.a(new_n283), .b(new_n280), .c(new_n281), .out0(new_n284));
  norp02aa1n02x5               g189(.a(new_n282), .b(new_n284), .o1(\s[29] ));
  xorb03aa1n02x5               g190(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g191(.a(new_n281), .b(new_n263), .c(new_n267), .out0(new_n287));
  oai012aa1n02x5               g192(.a(new_n287), .b(new_n272), .c(new_n268), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[29] ), .b(\b[28] ), .c(new_n280), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[29] ), .b(\a[30] ), .out0(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n02x5               g196(.a(new_n287), .b(new_n260), .c(new_n258), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n02x5               g198(.a(new_n291), .b(new_n293), .o1(\s[30] ));
  nano23aa1n02x4               g199(.a(new_n290), .b(new_n281), .c(new_n267), .d(new_n263), .out0(new_n295));
  aobi12aa1n02x5               g200(.a(new_n295), .b(new_n260), .c(new_n258), .out0(new_n296));
  oao003aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .c(new_n289), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[30] ), .b(\a[31] ), .out0(new_n298));
  nano22aa1n02x4               g203(.a(new_n296), .b(new_n297), .c(new_n298), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n295), .b(new_n272), .c(new_n268), .o1(new_n300));
  aoi012aa1n02x5               g205(.a(new_n298), .b(new_n300), .c(new_n297), .o1(new_n301));
  norp02aa1n02x5               g206(.a(new_n301), .b(new_n299), .o1(\s[31] ));
  xnrb03aa1n02x5               g207(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g208(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n304));
  xorb03aa1n02x5               g209(.a(new_n304), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g210(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoib12aa1n02x5               g211(.a(new_n120), .b(new_n109), .c(new_n116), .out0(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[5] ), .c(new_n118), .out0(\s[6] ));
  nona22aa1n02x4               g213(.a(new_n109), .b(new_n115), .c(new_n116), .out0(new_n309));
  nanp02aa1n02x5               g214(.a(new_n309), .b(new_n121), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g216(.a(new_n112), .b(new_n310), .c(new_n113), .o1(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g218(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


