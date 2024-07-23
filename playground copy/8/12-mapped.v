// Benchmark "adder" written by ABC on Thu Jul 11 11:27:43 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n147, new_n148, new_n149,
    new_n151, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n179, new_n180, new_n181,
    new_n182, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n211, new_n212, new_n213, new_n214,
    new_n215, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n227, new_n228, new_n229, new_n230,
    new_n231, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n246,
    new_n247, new_n248, new_n249, new_n250, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n302, new_n305,
    new_n306, new_n308, new_n310, new_n312;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1n02x4               g006(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .out0(new_n104));
  norp03aa1n02x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  aoi012aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  aoi012aa1n02x5               g019(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n115), .b(new_n114), .c(new_n109), .o1(new_n116));
  oai022aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  aob012aa1n02x5               g022(.a(new_n117), .b(\b[5] ), .c(\a[6] ), .out0(new_n118));
  aoi012aa1n02x5               g023(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n119), .b(new_n102), .c(new_n118), .o1(new_n120));
  xorc02aa1n02x5               g025(.a(\a[9] ), .b(\b[8] ), .out0(new_n121));
  aoai13aa1n02x5               g026(.a(new_n121), .b(new_n120), .c(new_n116), .d(new_n105), .o1(new_n122));
  xorc02aa1n02x5               g027(.a(\a[10] ), .b(\b[9] ), .out0(new_n123));
  xnbna2aa1n03x5               g028(.a(new_n123), .b(new_n122), .c(new_n97), .out0(\s[10] ));
  aobi12aa1n02x5               g029(.a(new_n123), .b(new_n122), .c(new_n97), .out0(new_n125));
  norp02aa1n02x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  oaoi03aa1n02x5               g033(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n129));
  oai012aa1n02x5               g034(.a(new_n128), .b(new_n125), .c(new_n129), .o1(new_n130));
  norp03aa1n02x5               g035(.a(new_n125), .b(new_n128), .c(new_n129), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n130), .b(new_n131), .out0(\s[11] ));
  norp02aa1n02x5               g037(.a(\b[11] ), .b(\a[12] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  nona22aa1n02x4               g040(.a(new_n130), .b(new_n135), .c(new_n126), .out0(new_n136));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n126), .clkout(new_n137));
  aobi12aa1n02x5               g042(.a(new_n135), .b(new_n130), .c(new_n137), .out0(new_n138));
  norb02aa1n02x5               g043(.a(new_n136), .b(new_n138), .out0(\s[12] ));
  nano23aa1n02x4               g044(.a(new_n126), .b(new_n133), .c(new_n134), .d(new_n127), .out0(new_n140));
  and003aa1n02x5               g045(.a(new_n140), .b(new_n123), .c(new_n121), .o(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n120), .c(new_n116), .d(new_n105), .o1(new_n142));
  aoi012aa1n02x5               g047(.a(new_n133), .b(new_n126), .c(new_n134), .o1(new_n143));
  aobi12aa1n02x5               g048(.a(new_n143), .b(new_n140), .c(new_n129), .out0(new_n144));
  nanp02aa1n02x5               g049(.a(new_n142), .b(new_n144), .o1(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n147), .b(new_n145), .c(new_n148), .o1(new_n149));
  xnrb03aa1n02x5               g054(.a(new_n149), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g055(.a(\b[14] ), .b(\a[15] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[14] ), .b(\a[15] ), .o1(new_n152));
  nanb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(new_n153));
  160nm_ficinv00aa1n08x5       g058(.clk(new_n153), .clkout(new_n154));
  norp02aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nano23aa1n02x4               g061(.a(new_n147), .b(new_n155), .c(new_n156), .d(new_n148), .out0(new_n157));
  oa0012aa1n02x5               g062(.a(new_n156), .b(new_n155), .c(new_n147), .o(new_n158));
  aoai13aa1n02x5               g063(.a(new_n154), .b(new_n158), .c(new_n145), .d(new_n157), .o1(new_n159));
  aoi112aa1n02x5               g064(.a(new_n154), .b(new_n158), .c(new_n145), .d(new_n157), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(\s[15] ));
  norp02aa1n02x5               g066(.a(\b[15] ), .b(\a[16] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  nanb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(new_n164));
  oai112aa1n02x5               g069(.a(new_n159), .b(new_n164), .c(\b[14] ), .d(\a[15] ), .o1(new_n165));
  oaoi13aa1n02x5               g070(.a(new_n164), .b(new_n159), .c(\a[15] ), .d(\b[14] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(\s[16] ));
  nona22aa1n02x4               g072(.a(new_n157), .b(new_n164), .c(new_n153), .out0(new_n168));
  nano32aa1n02x4               g073(.a(new_n168), .b(new_n140), .c(new_n123), .d(new_n121), .out0(new_n169));
  aoai13aa1n02x5               g074(.a(new_n169), .b(new_n120), .c(new_n116), .d(new_n105), .o1(new_n170));
  160nm_ficinv00aa1n08x5       g075(.clk(new_n162), .clkout(new_n171));
  nona22aa1n02x4               g076(.a(new_n158), .b(new_n164), .c(new_n153), .out0(new_n172));
  nanp02aa1n02x5               g077(.a(new_n151), .b(new_n163), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(new_n140), .b(new_n129), .o1(new_n174));
  aoi012aa1n02x5               g079(.a(new_n168), .b(new_n174), .c(new_n143), .o1(new_n175));
  nano32aa1n02x4               g080(.a(new_n175), .b(new_n173), .c(new_n172), .d(new_n171), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n170), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g083(.clk(\a[18] ), .clkout(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(\a[17] ), .clkout(new_n180));
  160nm_ficinv00aa1n08x5       g085(.clk(\b[16] ), .clkout(new_n181));
  oaoi03aa1n02x5               g086(.a(new_n180), .b(new_n181), .c(new_n177), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[17] ), .c(new_n179), .out0(\s[18] ));
  xroi22aa1d04x5               g088(.a(new_n180), .b(\b[16] ), .c(new_n179), .d(\b[17] ), .out0(new_n184));
  nanp02aa1n02x5               g089(.a(new_n181), .b(new_n180), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(\a[18] ), .b(\b[17] ), .c(new_n185), .o1(new_n186));
  norp02aa1n02x5               g091(.a(\b[18] ), .b(\a[19] ), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(\b[18] ), .b(\a[19] ), .o1(new_n188));
  norb02aa1n02x5               g093(.a(new_n188), .b(new_n187), .out0(new_n189));
  aoai13aa1n02x5               g094(.a(new_n189), .b(new_n186), .c(new_n177), .d(new_n184), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(new_n189), .b(new_n186), .c(new_n177), .d(new_n184), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n190), .b(new_n191), .out0(\s[19] ));
  xnrc02aa1n02x5               g097(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  nona22aa1n02x4               g101(.a(new_n190), .b(new_n196), .c(new_n187), .out0(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(new_n187), .clkout(new_n198));
  aobi12aa1n02x5               g103(.a(new_n196), .b(new_n190), .c(new_n198), .out0(new_n199));
  norb02aa1n02x5               g104(.a(new_n197), .b(new_n199), .out0(\s[20] ));
  nano23aa1n02x4               g105(.a(new_n187), .b(new_n194), .c(new_n195), .d(new_n188), .out0(new_n201));
  nanp02aa1n02x5               g106(.a(new_n184), .b(new_n201), .o1(new_n202));
  oai022aa1n02x5               g107(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n203));
  oaib12aa1n02x5               g108(.a(new_n203), .b(new_n179), .c(\b[17] ), .out0(new_n204));
  nona23aa1n02x4               g109(.a(new_n195), .b(new_n188), .c(new_n187), .d(new_n194), .out0(new_n205));
  aoi012aa1n02x5               g110(.a(new_n194), .b(new_n187), .c(new_n195), .o1(new_n206));
  oai012aa1n02x5               g111(.a(new_n206), .b(new_n205), .c(new_n204), .o1(new_n207));
  160nm_ficinv00aa1n08x5       g112(.clk(new_n207), .clkout(new_n208));
  aoai13aa1n02x5               g113(.a(new_n208), .b(new_n202), .c(new_n176), .d(new_n170), .o1(new_n209));
  xorb03aa1n02x5               g114(.a(new_n209), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g115(.a(\b[20] ), .b(\a[21] ), .o1(new_n211));
  xorc02aa1n02x5               g116(.a(\a[21] ), .b(\b[20] ), .out0(new_n212));
  xorc02aa1n02x5               g117(.a(\a[22] ), .b(\b[21] ), .out0(new_n213));
  aoi112aa1n02x5               g118(.a(new_n211), .b(new_n213), .c(new_n209), .d(new_n212), .o1(new_n214));
  aoai13aa1n02x5               g119(.a(new_n213), .b(new_n211), .c(new_n209), .d(new_n212), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g121(.clk(\a[21] ), .clkout(new_n217));
  160nm_ficinv00aa1n08x5       g122(.clk(\a[22] ), .clkout(new_n218));
  xroi22aa1d04x5               g123(.a(new_n217), .b(\b[20] ), .c(new_n218), .d(\b[21] ), .out0(new_n219));
  nanp03aa1n02x5               g124(.a(new_n219), .b(new_n184), .c(new_n201), .o1(new_n220));
  160nm_ficinv00aa1n08x5       g125(.clk(\b[21] ), .clkout(new_n221));
  oaoi03aa1n02x5               g126(.a(new_n218), .b(new_n221), .c(new_n211), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n222), .clkout(new_n223));
  aoi012aa1n02x5               g128(.a(new_n223), .b(new_n207), .c(new_n219), .o1(new_n224));
  aoai13aa1n02x5               g129(.a(new_n224), .b(new_n220), .c(new_n176), .d(new_n170), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g131(.a(\b[22] ), .b(\a[23] ), .o1(new_n227));
  xorc02aa1n02x5               g132(.a(\a[23] ), .b(\b[22] ), .out0(new_n228));
  xorc02aa1n02x5               g133(.a(\a[24] ), .b(\b[23] ), .out0(new_n229));
  aoi112aa1n02x5               g134(.a(new_n227), .b(new_n229), .c(new_n225), .d(new_n228), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n229), .b(new_n227), .c(new_n225), .d(new_n228), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(\s[24] ));
  and002aa1n02x5               g137(.a(new_n229), .b(new_n228), .o(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  nano32aa1n02x4               g139(.a(new_n234), .b(new_n219), .c(new_n184), .d(new_n201), .out0(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n206), .clkout(new_n236));
  aoai13aa1n02x5               g141(.a(new_n219), .b(new_n236), .c(new_n201), .d(new_n186), .o1(new_n237));
  norp02aa1n02x5               g142(.a(\b[23] ), .b(\a[24] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(\b[23] ), .b(\a[24] ), .o1(new_n239));
  aoi012aa1n02x5               g144(.a(new_n238), .b(new_n227), .c(new_n239), .o1(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n234), .c(new_n237), .d(new_n222), .o1(new_n241));
  xorc02aa1n02x5               g146(.a(\a[25] ), .b(\b[24] ), .out0(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n241), .c(new_n177), .d(new_n235), .o1(new_n243));
  aoi112aa1n02x5               g148(.a(new_n242), .b(new_n241), .c(new_n177), .d(new_n235), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n243), .b(new_n244), .out0(\s[25] ));
  norp02aa1n02x5               g150(.a(\b[24] ), .b(\a[25] ), .o1(new_n246));
  xorc02aa1n02x5               g151(.a(\a[26] ), .b(\b[25] ), .out0(new_n247));
  nona22aa1n02x4               g152(.a(new_n243), .b(new_n247), .c(new_n246), .out0(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n246), .clkout(new_n249));
  aobi12aa1n02x5               g154(.a(new_n247), .b(new_n243), .c(new_n249), .out0(new_n250));
  norb02aa1n02x5               g155(.a(new_n248), .b(new_n250), .out0(\s[26] ));
  and002aa1n02x5               g156(.a(new_n247), .b(new_n242), .o(new_n252));
  nano22aa1n02x4               g157(.a(new_n220), .b(new_n233), .c(new_n252), .out0(new_n253));
  nanp02aa1n02x5               g158(.a(new_n177), .b(new_n253), .o1(new_n254));
  oao003aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .c(new_n249), .carry(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n255), .clkout(new_n256));
  aoi012aa1n02x5               g161(.a(new_n256), .b(new_n241), .c(new_n252), .o1(new_n257));
  xorc02aa1n02x5               g162(.a(\a[27] ), .b(\b[26] ), .out0(new_n258));
  xnbna2aa1n03x5               g163(.a(new_n258), .b(new_n254), .c(new_n257), .out0(\s[27] ));
  norp02aa1n02x5               g164(.a(\b[26] ), .b(\a[27] ), .o1(new_n260));
  160nm_ficinv00aa1n08x5       g165(.clk(new_n260), .clkout(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n258), .clkout(new_n262));
  aoi012aa1n02x5               g167(.a(new_n262), .b(new_n254), .c(new_n257), .o1(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[27] ), .b(\a[28] ), .out0(new_n264));
  nano22aa1n02x4               g169(.a(new_n263), .b(new_n261), .c(new_n264), .out0(new_n265));
  aobi12aa1n02x5               g170(.a(new_n253), .b(new_n176), .c(new_n170), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n233), .b(new_n223), .c(new_n207), .d(new_n219), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n252), .clkout(new_n268));
  aoai13aa1n02x5               g173(.a(new_n255), .b(new_n268), .c(new_n267), .d(new_n240), .o1(new_n269));
  oai012aa1n02x5               g174(.a(new_n258), .b(new_n269), .c(new_n266), .o1(new_n270));
  aoi012aa1n02x5               g175(.a(new_n264), .b(new_n270), .c(new_n261), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n265), .o1(\s[28] ));
  norb02aa1n02x5               g177(.a(new_n258), .b(new_n264), .out0(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n273), .clkout(new_n274));
  aoi012aa1n02x5               g179(.a(new_n274), .b(new_n254), .c(new_n257), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[28] ), .b(\b[27] ), .c(new_n261), .carry(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[28] ), .b(\a[29] ), .out0(new_n277));
  nano22aa1n02x4               g182(.a(new_n275), .b(new_n276), .c(new_n277), .out0(new_n278));
  oai012aa1n02x5               g183(.a(new_n273), .b(new_n269), .c(new_n266), .o1(new_n279));
  aoi012aa1n02x5               g184(.a(new_n277), .b(new_n279), .c(new_n276), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n280), .b(new_n278), .o1(\s[29] ));
  xorb03aa1n02x5               g186(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g187(.a(new_n258), .b(new_n277), .c(new_n264), .out0(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n283), .clkout(new_n284));
  aoi012aa1n02x5               g189(.a(new_n284), .b(new_n254), .c(new_n257), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[29] ), .b(\b[28] ), .c(new_n276), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[29] ), .b(\a[30] ), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n285), .b(new_n286), .c(new_n287), .out0(new_n288));
  oai012aa1n02x5               g193(.a(new_n283), .b(new_n269), .c(new_n266), .o1(new_n289));
  aoi012aa1n02x5               g194(.a(new_n287), .b(new_n289), .c(new_n286), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n290), .b(new_n288), .o1(\s[30] ));
  norb02aa1n02x5               g196(.a(new_n283), .b(new_n287), .out0(new_n292));
  160nm_ficinv00aa1n08x5       g197(.clk(new_n292), .clkout(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n254), .c(new_n257), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[30] ), .b(\b[29] ), .c(new_n286), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[30] ), .b(\a[31] ), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n294), .b(new_n295), .c(new_n296), .out0(new_n297));
  oai012aa1n02x5               g202(.a(new_n292), .b(new_n269), .c(new_n266), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n296), .b(new_n298), .c(new_n295), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n299), .b(new_n297), .o1(\s[31] ));
  xnrb03aa1n02x5               g205(.a(new_n109), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g206(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n302));
  xorb03aa1n02x5               g207(.a(new_n302), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g208(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  160nm_ficinv00aa1n08x5       g209(.clk(new_n116), .clkout(new_n305));
  oaoi03aa1n02x5               g210(.a(\a[5] ), .b(\b[4] ), .c(new_n305), .o1(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai013aa1n02x4               g212(.a(new_n118), .b(new_n305), .c(new_n103), .d(new_n104), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g214(.a(new_n100), .b(new_n308), .c(new_n101), .o1(new_n310));
  xnrb03aa1n02x5               g215(.a(new_n310), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi112aa1n02x5               g216(.a(new_n120), .b(new_n121), .c(new_n116), .d(new_n105), .o1(new_n312));
  norb02aa1n02x5               g217(.a(new_n122), .b(new_n312), .out0(\s[9] ));
endmodule

