// Benchmark "adder" written by ABC on Thu Jul 11 12:45:01 2024

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
    new_n133, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n147, new_n148, new_n149,
    new_n151, new_n152, new_n153, new_n154, new_n156, new_n157, new_n158,
    new_n159, new_n160, new_n161, new_n162, new_n164, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n174,
    new_n175, new_n176, new_n177, new_n179, new_n180, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n197, new_n198, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n205, new_n206, new_n207,
    new_n208, new_n209, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n220, new_n221, new_n222, new_n223,
    new_n224, new_n226, new_n227, new_n228, new_n229, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n247, new_n248, new_n249, new_n250, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n303,
    new_n306, new_n308, new_n310;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  aoi012aa1n02x5               g011(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n107));
  oai012aa1n02x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1n02x4               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  norp03aa1n02x5               g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  160nm_ficinv00aa1n08x5       g021(.clk(\a[5] ), .clkout(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\b[4] ), .clkout(new_n118));
  nanp02aa1n02x5               g023(.a(new_n118), .b(new_n117), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  aoi012aa1n02x5               g025(.a(new_n109), .b(new_n111), .c(new_n110), .o1(new_n121));
  oaib12aa1n02x5               g026(.a(new_n121), .b(new_n113), .c(new_n120), .out0(new_n122));
  xorc02aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n124));
  norp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n97), .out0(\s[10] ));
  norp02aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  norp02aa1n02x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  nona22aa1n02x4               g037(.a(new_n124), .b(new_n125), .c(new_n132), .out0(new_n133));
  xobna2aa1n03x5               g038(.a(new_n131), .b(new_n133), .c(new_n126), .out0(\s[11] ));
  aoi013aa1n02x4               g039(.a(new_n129), .b(new_n133), .c(new_n131), .d(new_n126), .o1(new_n135));
  xnrb03aa1n02x5               g040(.a(new_n135), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nano23aa1n02x4               g043(.a(new_n129), .b(new_n137), .c(new_n138), .d(new_n130), .out0(new_n139));
  and003aa1n02x5               g044(.a(new_n139), .b(new_n127), .c(new_n123), .o(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n141));
  oaoi03aa1n02x5               g046(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n142));
  aoi012aa1n02x5               g047(.a(new_n137), .b(new_n129), .c(new_n138), .o1(new_n143));
  aobi12aa1n02x5               g048(.a(new_n143), .b(new_n139), .c(new_n142), .out0(new_n144));
  xorc02aa1n02x5               g049(.a(\a[13] ), .b(\b[12] ), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n141), .c(new_n144), .out0(\s[13] ));
  orn002aa1n02x5               g051(.a(\a[13] ), .b(\b[12] ), .o(new_n147));
  aob012aa1n02x5               g052(.a(new_n145), .b(new_n141), .c(new_n144), .out0(new_n148));
  xorc02aa1n02x5               g053(.a(\a[14] ), .b(\b[13] ), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n148), .c(new_n147), .out0(\s[14] ));
  nanp02aa1n02x5               g055(.a(new_n149), .b(new_n145), .o1(new_n151));
  oaoi03aa1n02x5               g056(.a(\a[14] ), .b(\b[13] ), .c(new_n147), .o1(new_n152));
  160nm_ficinv00aa1n08x5       g057(.clk(new_n152), .clkout(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n151), .c(new_n141), .d(new_n144), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  norp02aa1n02x5               g062(.a(\b[15] ), .b(\a[16] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[15] ), .b(\a[16] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  aoi112aa1n02x5               g065(.a(new_n160), .b(new_n156), .c(new_n154), .d(new_n157), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n160), .b(new_n156), .c(new_n154), .d(new_n157), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(\s[16] ));
  nano23aa1n02x4               g068(.a(new_n156), .b(new_n158), .c(new_n159), .d(new_n157), .out0(new_n164));
  nanp03aa1n02x5               g069(.a(new_n164), .b(new_n145), .c(new_n149), .o1(new_n165));
  nano32aa1n02x4               g070(.a(new_n165), .b(new_n139), .c(new_n127), .d(new_n123), .out0(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n167));
  160nm_fiao0012aa1n02p5x5     g072(.a(new_n158), .b(new_n156), .c(new_n159), .o(new_n168));
  aoi012aa1n02x5               g073(.a(new_n168), .b(new_n164), .c(new_n152), .o1(new_n169));
  160nm_ficinv00aa1n08x5       g074(.clk(new_n169), .clkout(new_n170));
  oab012aa1n02x4               g075(.a(new_n170), .b(new_n144), .c(new_n165), .out0(new_n171));
  nanp02aa1n02x5               g076(.a(new_n167), .b(new_n171), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g078(.clk(\a[18] ), .clkout(new_n174));
  160nm_ficinv00aa1n08x5       g079(.clk(\a[17] ), .clkout(new_n175));
  160nm_ficinv00aa1n08x5       g080(.clk(\b[16] ), .clkout(new_n176));
  oaoi03aa1n02x5               g081(.a(new_n175), .b(new_n176), .c(new_n172), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[17] ), .c(new_n174), .out0(\s[18] ));
  xroi22aa1d04x5               g083(.a(new_n175), .b(\b[16] ), .c(new_n174), .d(\b[17] ), .out0(new_n179));
  oai022aa1n02x5               g084(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n180));
  oaib12aa1n02x5               g085(.a(new_n180), .b(new_n174), .c(\b[17] ), .out0(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(new_n181), .clkout(new_n182));
  norp02aa1n02x5               g087(.a(\b[18] ), .b(\a[19] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(\b[18] ), .b(\a[19] ), .o1(new_n184));
  norb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  aoai13aa1n02x5               g090(.a(new_n185), .b(new_n182), .c(new_n172), .d(new_n179), .o1(new_n186));
  aoi112aa1n02x5               g091(.a(new_n185), .b(new_n182), .c(new_n172), .d(new_n179), .o1(new_n187));
  norb02aa1n02x5               g092(.a(new_n186), .b(new_n187), .out0(\s[19] ));
  xnrc02aa1n02x5               g093(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g094(.a(\b[19] ), .b(\a[20] ), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(\b[19] ), .b(\a[20] ), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  nona22aa1n02x4               g097(.a(new_n186), .b(new_n192), .c(new_n183), .out0(new_n193));
  160nm_ficinv00aa1n08x5       g098(.clk(new_n183), .clkout(new_n194));
  aobi12aa1n02x5               g099(.a(new_n192), .b(new_n186), .c(new_n194), .out0(new_n195));
  norb02aa1n02x5               g100(.a(new_n193), .b(new_n195), .out0(\s[20] ));
  nano23aa1n02x4               g101(.a(new_n183), .b(new_n190), .c(new_n191), .d(new_n184), .out0(new_n197));
  nanp02aa1n02x5               g102(.a(new_n179), .b(new_n197), .o1(new_n198));
  nona23aa1n02x4               g103(.a(new_n191), .b(new_n184), .c(new_n183), .d(new_n190), .out0(new_n199));
  aoi012aa1n02x5               g104(.a(new_n190), .b(new_n183), .c(new_n191), .o1(new_n200));
  oai012aa1n02x5               g105(.a(new_n200), .b(new_n199), .c(new_n181), .o1(new_n201));
  160nm_ficinv00aa1n08x5       g106(.clk(new_n201), .clkout(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n198), .c(new_n167), .d(new_n171), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g109(.a(\b[20] ), .b(\a[21] ), .o1(new_n205));
  xorc02aa1n02x5               g110(.a(\a[21] ), .b(\b[20] ), .out0(new_n206));
  xorc02aa1n02x5               g111(.a(\a[22] ), .b(\b[21] ), .out0(new_n207));
  aoi112aa1n02x5               g112(.a(new_n205), .b(new_n207), .c(new_n203), .d(new_n206), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n207), .b(new_n205), .c(new_n203), .d(new_n206), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n209), .b(new_n208), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g115(.clk(\a[21] ), .clkout(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(\a[22] ), .clkout(new_n212));
  xroi22aa1d04x5               g117(.a(new_n211), .b(\b[20] ), .c(new_n212), .d(\b[21] ), .out0(new_n213));
  nanp03aa1n02x5               g118(.a(new_n213), .b(new_n179), .c(new_n197), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(\b[21] ), .clkout(new_n215));
  oao003aa1n02x5               g120(.a(new_n212), .b(new_n215), .c(new_n205), .carry(new_n216));
  aoi012aa1n02x5               g121(.a(new_n216), .b(new_n201), .c(new_n213), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n214), .c(new_n167), .d(new_n171), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g124(.a(\b[22] ), .b(\a[23] ), .o1(new_n220));
  xorc02aa1n02x5               g125(.a(\a[23] ), .b(\b[22] ), .out0(new_n221));
  xorc02aa1n02x5               g126(.a(\a[24] ), .b(\b[23] ), .out0(new_n222));
  aoi112aa1n02x5               g127(.a(new_n220), .b(new_n222), .c(new_n218), .d(new_n221), .o1(new_n223));
  aoai13aa1n02x5               g128(.a(new_n222), .b(new_n220), .c(new_n218), .d(new_n221), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(\s[24] ));
  nano23aa1n02x4               g130(.a(new_n102), .b(new_n104), .c(new_n105), .d(new_n103), .out0(new_n226));
  nanb02aa1n02x5               g131(.a(new_n101), .b(new_n226), .out0(new_n227));
  nano23aa1n02x4               g132(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n228));
  nona22aa1n02x4               g133(.a(new_n228), .b(new_n114), .c(new_n115), .out0(new_n229));
  aobi12aa1n02x5               g134(.a(new_n121), .b(new_n228), .c(new_n120), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n229), .c(new_n227), .d(new_n107), .o1(new_n231));
  oai012aa1n02x5               g136(.a(new_n169), .b(new_n144), .c(new_n165), .o1(new_n232));
  and002aa1n02x5               g137(.a(new_n222), .b(new_n221), .o(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  nano32aa1n02x4               g139(.a(new_n234), .b(new_n213), .c(new_n179), .d(new_n197), .out0(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n232), .c(new_n231), .d(new_n166), .o1(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n200), .clkout(new_n237));
  aoai13aa1n02x5               g142(.a(new_n213), .b(new_n237), .c(new_n197), .d(new_n182), .o1(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n216), .clkout(new_n239));
  aoi112aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n240));
  oab012aa1n02x4               g145(.a(new_n240), .b(\a[24] ), .c(\b[23] ), .out0(new_n241));
  aoai13aa1n02x5               g146(.a(new_n241), .b(new_n234), .c(new_n238), .d(new_n239), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n242), .clkout(new_n243));
  xnrc02aa1n02x5               g148(.a(\b[24] ), .b(\a[25] ), .out0(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n244), .clkout(new_n245));
  xnbna2aa1n03x5               g150(.a(new_n245), .b(new_n236), .c(new_n243), .out0(\s[25] ));
  aoai13aa1n02x5               g151(.a(new_n245), .b(new_n242), .c(new_n172), .d(new_n235), .o1(new_n247));
  xnrc02aa1n02x5               g152(.a(\b[25] ), .b(\a[26] ), .out0(new_n248));
  oai112aa1n02x5               g153(.a(new_n247), .b(new_n248), .c(\b[24] ), .d(\a[25] ), .o1(new_n249));
  oaoi13aa1n02x5               g154(.a(new_n248), .b(new_n247), .c(\a[25] ), .d(\b[24] ), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(\s[26] ));
  norp02aa1n02x5               g156(.a(new_n248), .b(new_n244), .o1(new_n252));
  nano22aa1n02x4               g157(.a(new_n214), .b(new_n233), .c(new_n252), .out0(new_n253));
  aoai13aa1n02x5               g158(.a(new_n253), .b(new_n232), .c(new_n231), .d(new_n166), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(new_n242), .b(new_n252), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n256));
  oab012aa1n02x4               g161(.a(new_n256), .b(\a[26] ), .c(\b[25] ), .out0(new_n257));
  xorc02aa1n02x5               g162(.a(\a[27] ), .b(\b[26] ), .out0(new_n258));
  160nm_ficinv00aa1n08x5       g163(.clk(new_n258), .clkout(new_n259));
  aoi013aa1n02x4               g164(.a(new_n259), .b(new_n254), .c(new_n255), .d(new_n257), .o1(new_n260));
  aobi12aa1n02x5               g165(.a(new_n253), .b(new_n167), .c(new_n171), .out0(new_n261));
  aoai13aa1n02x5               g166(.a(new_n233), .b(new_n216), .c(new_n201), .d(new_n213), .o1(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(new_n252), .clkout(new_n263));
  aoai13aa1n02x5               g168(.a(new_n257), .b(new_n263), .c(new_n262), .d(new_n241), .o1(new_n264));
  norp03aa1n02x5               g169(.a(new_n264), .b(new_n261), .c(new_n258), .o1(new_n265));
  norp02aa1n02x5               g170(.a(new_n260), .b(new_n265), .o1(\s[27] ));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n267), .clkout(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[27] ), .b(\a[28] ), .out0(new_n269));
  nano22aa1n02x4               g174(.a(new_n260), .b(new_n268), .c(new_n269), .out0(new_n270));
  oai012aa1n02x5               g175(.a(new_n258), .b(new_n264), .c(new_n261), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n269), .b(new_n271), .c(new_n268), .o1(new_n272));
  norp02aa1n02x5               g177(.a(new_n272), .b(new_n270), .o1(\s[28] ));
  norb02aa1n02x5               g178(.a(new_n258), .b(new_n269), .out0(new_n274));
  oai012aa1n02x5               g179(.a(new_n274), .b(new_n264), .c(new_n261), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[28] ), .b(\b[27] ), .c(new_n268), .carry(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[28] ), .b(\a[29] ), .out0(new_n277));
  aoi012aa1n02x5               g182(.a(new_n277), .b(new_n275), .c(new_n276), .o1(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n274), .clkout(new_n279));
  aoi013aa1n02x4               g184(.a(new_n279), .b(new_n254), .c(new_n255), .d(new_n257), .o1(new_n280));
  nano22aa1n02x4               g185(.a(new_n280), .b(new_n276), .c(new_n277), .out0(new_n281));
  norp02aa1n02x5               g186(.a(new_n278), .b(new_n281), .o1(\s[29] ));
  xorb03aa1n02x5               g187(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g188(.a(new_n258), .b(new_n277), .c(new_n269), .out0(new_n284));
  oai012aa1n02x5               g189(.a(new_n284), .b(new_n264), .c(new_n261), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[29] ), .b(\b[28] ), .c(new_n276), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[29] ), .b(\a[30] ), .out0(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n285), .c(new_n286), .o1(new_n288));
  160nm_ficinv00aa1n08x5       g193(.clk(new_n284), .clkout(new_n289));
  aoi013aa1n02x4               g194(.a(new_n289), .b(new_n254), .c(new_n255), .d(new_n257), .o1(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n286), .c(new_n287), .out0(new_n291));
  norp02aa1n02x5               g196(.a(new_n288), .b(new_n291), .o1(\s[30] ));
  norb02aa1n02x5               g197(.a(new_n284), .b(new_n287), .out0(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n293), .clkout(new_n294));
  aoi013aa1n02x4               g199(.a(new_n294), .b(new_n254), .c(new_n255), .d(new_n257), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[30] ), .b(\b[29] ), .c(new_n286), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[30] ), .b(\a[31] ), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n295), .b(new_n296), .c(new_n297), .out0(new_n298));
  oai012aa1n02x5               g203(.a(new_n293), .b(new_n264), .c(new_n261), .o1(new_n299));
  aoi012aa1n02x5               g204(.a(new_n297), .b(new_n299), .c(new_n296), .o1(new_n300));
  norp02aa1n02x5               g205(.a(new_n300), .b(new_n298), .o1(\s[31] ));
  xnrb03aa1n02x5               g206(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g207(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g209(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g210(.a(new_n117), .b(new_n118), .c(new_n108), .o1(new_n306));
  xnrb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g212(.a(\a[6] ), .b(\b[5] ), .c(new_n306), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g214(.a(new_n111), .b(new_n308), .c(new_n112), .o1(new_n310));
  xnrb03aa1n02x5               g215(.a(new_n310), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g216(.a(new_n231), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


