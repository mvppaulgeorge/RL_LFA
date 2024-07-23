// Benchmark "adder" written by ABC on Thu Jul 11 11:32:30 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n148, new_n149,
    new_n150, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n308, new_n309, new_n311, new_n312, new_n314;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1n02x4               g006(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .out0(new_n104));
  norp03aa1n02x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  aoi012aa1n02x5               g019(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n115), .b(new_n114), .c(new_n109), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(new_n116), .b(new_n105), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[6] ), .clkout(new_n118));
  oai022aa1n02x5               g023(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n119));
  oaib12aa1n02x5               g024(.a(new_n119), .b(new_n118), .c(\b[5] ), .out0(new_n120));
  160nm_fiao0012aa1n02p5x5     g025(.a(new_n98), .b(new_n100), .c(new_n99), .o(new_n121));
  oabi12aa1n02x5               g026(.a(new_n121), .b(new_n102), .c(new_n120), .out0(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(new_n122), .clkout(new_n123));
  nanp02aa1n02x5               g028(.a(new_n117), .b(new_n123), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n97), .b(new_n124), .c(new_n125), .o1(new_n126));
  xnrb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  oa0012aa1n02x5               g034(.a(new_n129), .b(new_n128), .c(new_n97), .o(new_n130));
  nano23aa1n02x4               g035(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n125), .out0(new_n131));
  aoi012aa1n02x5               g036(.a(new_n130), .b(new_n124), .c(new_n131), .o1(new_n132));
  xnrb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g038(.a(\a[11] ), .b(\b[10] ), .c(new_n132), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nano23aa1n02x4               g044(.a(new_n136), .b(new_n138), .c(new_n139), .d(new_n137), .out0(new_n140));
  and002aa1n02x5               g045(.a(new_n140), .b(new_n131), .o(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n122), .c(new_n116), .d(new_n105), .o1(new_n142));
  aoi012aa1n02x5               g047(.a(new_n138), .b(new_n136), .c(new_n139), .o1(new_n143));
  160nm_ficinv00aa1n08x5       g048(.clk(new_n143), .clkout(new_n144));
  aoi012aa1n02x5               g049(.a(new_n144), .b(new_n140), .c(new_n130), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(new_n142), .b(new_n145), .o1(new_n146));
  xorb03aa1n02x5               g051(.a(new_n146), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g052(.clk(\a[13] ), .clkout(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(\b[12] ), .clkout(new_n149));
  oaoi03aa1n02x5               g054(.a(new_n148), .b(new_n149), .c(new_n146), .o1(new_n150));
  xnrb03aa1n02x5               g055(.a(new_n150), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norp02aa1n02x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nona23aa1n02x4               g060(.a(new_n155), .b(new_n153), .c(new_n152), .d(new_n154), .out0(new_n156));
  aoai13aa1n02x5               g061(.a(new_n155), .b(new_n154), .c(new_n148), .d(new_n149), .o1(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n156), .c(new_n142), .d(new_n145), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  160nm_ficinv00aa1n08x5       g064(.clk(\a[15] ), .clkout(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(\b[14] ), .clkout(new_n161));
  oaoi03aa1n02x5               g066(.a(new_n160), .b(new_n161), .c(new_n158), .o1(new_n162));
  norp02aa1n02x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  160nm_ficinv00aa1n08x5       g068(.clk(new_n163), .clkout(new_n164));
  nanp02aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n162), .b(new_n165), .c(new_n164), .out0(\s[16] ));
  nanp02aa1n02x5               g071(.a(new_n161), .b(new_n160), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(new_n167), .b(new_n168), .o1(new_n169));
  nano23aa1n02x4               g074(.a(new_n152), .b(new_n154), .c(new_n155), .d(new_n153), .out0(new_n170));
  nanb02aa1n02x5               g075(.a(new_n163), .b(new_n165), .out0(new_n171));
  nona22aa1n02x4               g076(.a(new_n170), .b(new_n171), .c(new_n169), .out0(new_n172));
  nano22aa1n02x4               g077(.a(new_n172), .b(new_n131), .c(new_n140), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n122), .c(new_n116), .d(new_n105), .o1(new_n174));
  norp03aa1n02x5               g079(.a(new_n157), .b(new_n171), .c(new_n169), .o1(new_n175));
  160nm_ficinv00aa1n08x5       g080(.clk(new_n175), .clkout(new_n176));
  aoi112aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n177));
  160nm_ficinv00aa1n08x5       g082(.clk(new_n177), .clkout(new_n178));
  nanp02aa1n02x5               g083(.a(new_n140), .b(new_n130), .o1(new_n179));
  aoi012aa1n02x5               g084(.a(new_n172), .b(new_n179), .c(new_n143), .o1(new_n180));
  nano32aa1n02x4               g085(.a(new_n180), .b(new_n176), .c(new_n178), .d(new_n164), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n181), .b(new_n174), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[18] ), .clkout(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[17] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\b[16] ), .clkout(new_n186));
  oaoi03aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  xroi22aa1d04x5               g093(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n189));
  nanp02aa1n02x5               g094(.a(new_n186), .b(new_n185), .o1(new_n190));
  oaoi03aa1n02x5               g095(.a(\a[18] ), .b(\b[17] ), .c(new_n190), .o1(new_n191));
  norp02aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  aoai13aa1n02x5               g099(.a(new_n194), .b(new_n191), .c(new_n182), .d(new_n189), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n194), .b(new_n191), .c(new_n182), .d(new_n189), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  nona22aa1n02x4               g106(.a(new_n195), .b(new_n201), .c(new_n192), .out0(new_n202));
  160nm_ficinv00aa1n08x5       g107(.clk(new_n192), .clkout(new_n203));
  aobi12aa1n02x5               g108(.a(new_n201), .b(new_n195), .c(new_n203), .out0(new_n204));
  norb02aa1n02x5               g109(.a(new_n202), .b(new_n204), .out0(\s[20] ));
  nano23aa1n02x4               g110(.a(new_n192), .b(new_n199), .c(new_n200), .d(new_n193), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n189), .b(new_n206), .o1(new_n207));
  oai022aa1n02x5               g112(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n208));
  oaib12aa1n02x5               g113(.a(new_n208), .b(new_n184), .c(\b[17] ), .out0(new_n209));
  nona23aa1n02x4               g114(.a(new_n200), .b(new_n193), .c(new_n192), .d(new_n199), .out0(new_n210));
  aoi012aa1n02x5               g115(.a(new_n199), .b(new_n192), .c(new_n200), .o1(new_n211));
  oai012aa1n02x5               g116(.a(new_n211), .b(new_n210), .c(new_n209), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n207), .c(new_n181), .d(new_n174), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  aoi112aa1n02x5               g123(.a(new_n216), .b(new_n218), .c(new_n214), .d(new_n217), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n218), .b(new_n216), .c(new_n214), .d(new_n217), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g126(.clk(\a[21] ), .clkout(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[22] ), .clkout(new_n223));
  xroi22aa1d04x5               g128(.a(new_n222), .b(\b[20] ), .c(new_n223), .d(\b[21] ), .out0(new_n224));
  nanp03aa1n02x5               g129(.a(new_n224), .b(new_n189), .c(new_n206), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(\b[21] ), .clkout(new_n226));
  oaoi03aa1n02x5               g131(.a(new_n223), .b(new_n226), .c(new_n216), .o1(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n227), .clkout(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n212), .c(new_n224), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n225), .c(new_n181), .d(new_n174), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  and002aa1n02x5               g142(.a(new_n234), .b(new_n233), .o(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  nano32aa1n02x4               g144(.a(new_n239), .b(new_n224), .c(new_n189), .d(new_n206), .out0(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n211), .clkout(new_n241));
  aoai13aa1n02x5               g146(.a(new_n224), .b(new_n241), .c(new_n206), .d(new_n191), .o1(new_n242));
  norp02aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(\b[23] ), .b(\a[24] ), .o1(new_n244));
  aoi012aa1n02x5               g149(.a(new_n243), .b(new_n232), .c(new_n244), .o1(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n239), .c(new_n242), .d(new_n227), .o1(new_n246));
  xorc02aa1n02x5               g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n247), .b(new_n246), .c(new_n182), .d(new_n240), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n247), .b(new_n246), .c(new_n182), .d(new_n240), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n248), .b(new_n249), .out0(\s[25] ));
  norp02aa1n02x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[26] ), .b(\b[25] ), .out0(new_n252));
  nona22aa1n02x4               g157(.a(new_n248), .b(new_n252), .c(new_n251), .out0(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n251), .clkout(new_n254));
  aobi12aa1n02x5               g159(.a(new_n252), .b(new_n248), .c(new_n254), .out0(new_n255));
  norb02aa1n02x5               g160(.a(new_n253), .b(new_n255), .out0(\s[26] ));
  norp03aa1n02x5               g161(.a(new_n156), .b(new_n171), .c(new_n169), .o1(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n144), .c(new_n130), .d(new_n140), .o1(new_n258));
  nona32aa1n02x4               g163(.a(new_n258), .b(new_n177), .c(new_n175), .d(new_n163), .out0(new_n259));
  and002aa1n02x5               g164(.a(new_n252), .b(new_n247), .o(new_n260));
  nano22aa1n02x4               g165(.a(new_n225), .b(new_n238), .c(new_n260), .out0(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n259), .c(new_n124), .d(new_n173), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .c(new_n254), .carry(new_n263));
  aobi12aa1n02x5               g168(.a(new_n263), .b(new_n246), .c(new_n260), .out0(new_n264));
  xorc02aa1n02x5               g169(.a(\a[27] ), .b(\b[26] ), .out0(new_n265));
  xnbna2aa1n03x5               g170(.a(new_n265), .b(new_n264), .c(new_n262), .out0(\s[27] ));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n267), .clkout(new_n268));
  aobi12aa1n02x5               g173(.a(new_n265), .b(new_n264), .c(new_n262), .out0(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[27] ), .b(\a[28] ), .out0(new_n270));
  nano22aa1n02x4               g175(.a(new_n269), .b(new_n268), .c(new_n270), .out0(new_n271));
  aobi12aa1n02x5               g176(.a(new_n261), .b(new_n181), .c(new_n174), .out0(new_n272));
  aoai13aa1n02x5               g177(.a(new_n238), .b(new_n228), .c(new_n212), .d(new_n224), .o1(new_n273));
  160nm_ficinv00aa1n08x5       g178(.clk(new_n260), .clkout(new_n274));
  aoai13aa1n02x5               g179(.a(new_n263), .b(new_n274), .c(new_n273), .d(new_n245), .o1(new_n275));
  oai012aa1n02x5               g180(.a(new_n265), .b(new_n275), .c(new_n272), .o1(new_n276));
  aoi012aa1n02x5               g181(.a(new_n270), .b(new_n276), .c(new_n268), .o1(new_n277));
  norp02aa1n02x5               g182(.a(new_n277), .b(new_n271), .o1(\s[28] ));
  norb02aa1n02x5               g183(.a(new_n265), .b(new_n270), .out0(new_n279));
  aobi12aa1n02x5               g184(.a(new_n279), .b(new_n264), .c(new_n262), .out0(new_n280));
  oao003aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .c(new_n268), .carry(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  nano22aa1n02x4               g187(.a(new_n280), .b(new_n281), .c(new_n282), .out0(new_n283));
  oai012aa1n02x5               g188(.a(new_n279), .b(new_n275), .c(new_n272), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n282), .b(new_n284), .c(new_n281), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n285), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g191(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g192(.a(new_n265), .b(new_n282), .c(new_n270), .out0(new_n288));
  aobi12aa1n02x5               g193(.a(new_n288), .b(new_n264), .c(new_n262), .out0(new_n289));
  oao003aa1n02x5               g194(.a(\a[29] ), .b(\b[28] ), .c(new_n281), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[29] ), .b(\a[30] ), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n289), .b(new_n290), .c(new_n291), .out0(new_n292));
  oai012aa1n02x5               g197(.a(new_n288), .b(new_n275), .c(new_n272), .o1(new_n293));
  aoi012aa1n02x5               g198(.a(new_n291), .b(new_n293), .c(new_n290), .o1(new_n294));
  norp02aa1n02x5               g199(.a(new_n294), .b(new_n292), .o1(\s[30] ));
  xnrc02aa1n02x5               g200(.a(\b[30] ), .b(\a[31] ), .out0(new_n296));
  norb02aa1n02x5               g201(.a(new_n288), .b(new_n291), .out0(new_n297));
  aobi12aa1n02x5               g202(.a(new_n297), .b(new_n264), .c(new_n262), .out0(new_n298));
  oao003aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .c(new_n290), .carry(new_n299));
  nano22aa1n02x4               g204(.a(new_n298), .b(new_n296), .c(new_n299), .out0(new_n300));
  oai012aa1n02x5               g205(.a(new_n297), .b(new_n275), .c(new_n272), .o1(new_n301));
  aoi012aa1n02x5               g206(.a(new_n296), .b(new_n301), .c(new_n299), .o1(new_n302));
  norp02aa1n02x5               g207(.a(new_n302), .b(new_n300), .o1(\s[31] ));
  xnrb03aa1n02x5               g208(.a(new_n109), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g209(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g211(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g212(.a(\a[5] ), .b(\b[4] ), .o(new_n308));
  nanb02aa1n02x5               g213(.a(new_n104), .b(new_n116), .out0(new_n309));
  xobna2aa1n03x5               g214(.a(new_n103), .b(new_n309), .c(new_n308), .out0(\s[6] ));
  norp02aa1n02x5               g215(.a(new_n104), .b(new_n103), .o1(new_n311));
  aobi12aa1n02x5               g216(.a(new_n120), .b(new_n116), .c(new_n311), .out0(new_n312));
  xnrb03aa1n02x5               g217(.a(new_n312), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g218(.a(\a[7] ), .b(\b[6] ), .c(new_n312), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g220(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


