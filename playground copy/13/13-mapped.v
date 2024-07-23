// Benchmark "adder" written by ABC on Thu Jul 11 11:50:16 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n339, new_n342, new_n344, new_n346;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[9] ), .clkout(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\b[8] ), .clkout(new_n98));
  and002aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[3] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\b[2] ), .clkout(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  160nm_ficinv00aa1n08x5       g009(.clk(\a[2] ), .clkout(new_n105));
  160nm_ficinv00aa1n08x5       g010(.clk(\b[1] ), .clkout(new_n106));
  nanp02aa1n02x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  aob012aa1n02x5               g013(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(new_n109));
  160nm_ficinv00aa1n08x5       g014(.clk(\a[4] ), .clkout(new_n110));
  aboi22aa1n03x5               g015(.a(\b[3] ), .b(new_n110), .c(new_n100), .d(new_n101), .out0(new_n111));
  aoai13aa1n02x5               g016(.a(new_n111), .b(new_n104), .c(new_n109), .d(new_n107), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .out0(new_n118));
  orn002aa1n02x5               g023(.a(\a[5] ), .b(\b[4] ), .o(new_n119));
  nanp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nano22aa1n02x4               g025(.a(new_n118), .b(new_n119), .c(new_n120), .out0(new_n121));
  nona23aa1n02x4               g026(.a(new_n112), .b(new_n121), .c(new_n117), .d(new_n99), .out0(new_n122));
  oai022aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  aob012aa1n02x5               g028(.a(new_n123), .b(\b[5] ), .c(\a[6] ), .out0(new_n124));
  160nm_fiao0012aa1n02p5x5     g029(.a(new_n113), .b(new_n115), .c(new_n114), .o(new_n125));
  oabi12aa1n02x5               g030(.a(new_n125), .b(new_n117), .c(new_n124), .out0(new_n126));
  160nm_ficinv00aa1n08x5       g031(.clk(new_n126), .clkout(new_n127));
  nanp02aa1n02x5               g032(.a(new_n122), .b(new_n127), .o1(new_n128));
  oaoi03aa1n02x5               g033(.a(new_n97), .b(new_n98), .c(new_n128), .o1(new_n129));
  xnrb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n133), .clkout(new_n134));
  norp02aa1n02x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n135), .c(new_n97), .d(new_n98), .o1(new_n137));
  160nm_ficinv00aa1n08x5       g042(.clk(new_n137), .clkout(new_n138));
  norp02aa1n02x5               g043(.a(\b[8] ), .b(\a[9] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[8] ), .b(\a[9] ), .o1(new_n140));
  nano23aa1n02x4               g045(.a(new_n135), .b(new_n139), .c(new_n140), .d(new_n136), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n134), .b(new_n138), .c(new_n128), .d(new_n141), .o1(new_n142));
  aoi112aa1n02x5               g047(.a(new_n138), .b(new_n134), .c(new_n128), .d(new_n141), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n142), .b(new_n143), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g049(.clk(new_n131), .clkout(new_n145));
  norp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n148), .b(new_n142), .c(new_n145), .out0(\s[12] ));
  nano23aa1n02x4               g054(.a(new_n131), .b(new_n146), .c(new_n147), .d(new_n132), .out0(new_n150));
  nanp02aa1n02x5               g055(.a(new_n150), .b(new_n141), .o1(new_n151));
  nona23aa1n02x4               g056(.a(new_n147), .b(new_n132), .c(new_n131), .d(new_n146), .out0(new_n152));
  aoi012aa1n02x5               g057(.a(new_n146), .b(new_n131), .c(new_n147), .o1(new_n153));
  oai012aa1n02x5               g058(.a(new_n153), .b(new_n152), .c(new_n137), .o1(new_n154));
  160nm_ficinv00aa1n08x5       g059(.clk(new_n154), .clkout(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n151), .c(new_n122), .d(new_n127), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  aoi022aa1n02x5               g066(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n162), .b(new_n105), .c(new_n106), .o1(new_n163));
  oaoi13aa1n02x5               g068(.a(new_n99), .b(new_n111), .c(new_n163), .d(new_n104), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(new_n119), .b(new_n120), .o1(new_n165));
  norp03aa1n02x5               g070(.a(new_n117), .b(new_n118), .c(new_n165), .o1(new_n166));
  160nm_ficinv00aa1n08x5       g071(.clk(new_n151), .clkout(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n126), .c(new_n164), .d(new_n166), .o1(new_n168));
  norp02aa1n02x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nona23aa1n02x4               g075(.a(new_n170), .b(new_n159), .c(new_n158), .d(new_n169), .out0(new_n171));
  oai012aa1n02x5               g076(.a(new_n170), .b(new_n169), .c(new_n158), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n171), .c(new_n168), .d(new_n155), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  norp02aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanp02aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  aoi112aa1n02x5               g085(.a(new_n180), .b(new_n175), .c(new_n173), .d(new_n177), .o1(new_n181));
  160nm_ficinv00aa1n08x5       g086(.clk(new_n175), .clkout(new_n182));
  nano23aa1n02x4               g087(.a(new_n158), .b(new_n169), .c(new_n170), .d(new_n159), .out0(new_n183));
  160nm_ficinv00aa1n08x5       g088(.clk(new_n172), .clkout(new_n184));
  aoai13aa1n02x5               g089(.a(new_n177), .b(new_n184), .c(new_n156), .d(new_n183), .o1(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(new_n180), .clkout(new_n186));
  aoi012aa1n02x5               g091(.a(new_n186), .b(new_n185), .c(new_n182), .o1(new_n187));
  norp02aa1n02x5               g092(.a(new_n187), .b(new_n181), .o1(\s[16] ));
  nona23aa1n02x4               g093(.a(new_n179), .b(new_n176), .c(new_n175), .d(new_n178), .out0(new_n189));
  nona23aa1n02x4               g094(.a(new_n183), .b(new_n141), .c(new_n189), .d(new_n152), .out0(new_n190));
  norp02aa1n02x5               g095(.a(new_n189), .b(new_n171), .o1(new_n191));
  aoi012aa1n02x5               g096(.a(new_n178), .b(new_n175), .c(new_n179), .o1(new_n192));
  oai012aa1n02x5               g097(.a(new_n192), .b(new_n189), .c(new_n172), .o1(new_n193));
  aoi012aa1n02x5               g098(.a(new_n193), .b(new_n154), .c(new_n191), .o1(new_n194));
  aoai13aa1n02x5               g099(.a(new_n194), .b(new_n190), .c(new_n122), .d(new_n127), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nano32aa1n02x4               g101(.a(new_n151), .b(new_n180), .c(new_n177), .d(new_n183), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n126), .c(new_n164), .d(new_n166), .o1(new_n198));
  160nm_ficinv00aa1n08x5       g103(.clk(\a[17] ), .clkout(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(\b[16] ), .clkout(new_n200));
  nanp02aa1n02x5               g105(.a(new_n200), .b(new_n199), .o1(new_n201));
  and002aa1n02x5               g106(.a(\b[16] ), .b(\a[17] ), .o(new_n202));
  aoai13aa1n02x5               g107(.a(new_n201), .b(new_n202), .c(new_n198), .d(new_n194), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  norp02aa1n02x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nano23aa1n02x4               g111(.a(new_n205), .b(new_n202), .c(new_n201), .d(new_n206), .out0(new_n207));
  160nm_ficinv00aa1n08x5       g112(.clk(new_n207), .clkout(new_n208));
  aoai13aa1n02x5               g113(.a(new_n206), .b(new_n205), .c(new_n199), .d(new_n200), .o1(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n208), .c(new_n198), .d(new_n194), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  norp02aa1n02x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nanp02aa1n02x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nanb02aa1n02x5               g123(.a(new_n217), .b(new_n218), .out0(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(new_n219), .clkout(new_n220));
  aoi112aa1n02x5               g125(.a(new_n213), .b(new_n220), .c(new_n210), .d(new_n216), .o1(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(new_n213), .clkout(new_n222));
  oaoi03aa1n02x5               g127(.a(\a[18] ), .b(\b[17] ), .c(new_n201), .o1(new_n223));
  aoai13aa1n02x5               g128(.a(new_n216), .b(new_n223), .c(new_n195), .d(new_n207), .o1(new_n224));
  aoi012aa1n02x5               g129(.a(new_n219), .b(new_n224), .c(new_n222), .o1(new_n225));
  norp02aa1n02x5               g130(.a(new_n225), .b(new_n221), .o1(\s[20] ));
  nano23aa1n02x4               g131(.a(new_n213), .b(new_n217), .c(new_n218), .d(new_n214), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n207), .b(new_n227), .o1(new_n228));
  nona23aa1n02x4               g133(.a(new_n218), .b(new_n214), .c(new_n213), .d(new_n217), .out0(new_n229));
  aoi012aa1n02x5               g134(.a(new_n217), .b(new_n213), .c(new_n218), .o1(new_n230));
  oai012aa1n02x5               g135(.a(new_n230), .b(new_n229), .c(new_n209), .o1(new_n231));
  160nm_ficinv00aa1n08x5       g136(.clk(new_n231), .clkout(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n228), .c(new_n198), .d(new_n194), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  norp02aa1n02x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  aoi112aa1n02x5               g145(.a(new_n235), .b(new_n240), .c(new_n233), .d(new_n237), .o1(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n235), .clkout(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n228), .clkout(new_n243));
  aoai13aa1n02x5               g148(.a(new_n237), .b(new_n231), .c(new_n195), .d(new_n243), .o1(new_n244));
  160nm_ficinv00aa1n08x5       g149(.clk(new_n240), .clkout(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n244), .c(new_n242), .o1(new_n246));
  norp02aa1n02x5               g151(.a(new_n246), .b(new_n241), .o1(\s[22] ));
  nano23aa1n02x4               g152(.a(new_n235), .b(new_n238), .c(new_n239), .d(new_n236), .out0(new_n248));
  nano22aa1n02x4               g153(.a(new_n208), .b(new_n227), .c(new_n248), .out0(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n249), .clkout(new_n250));
  oaoi03aa1n02x5               g155(.a(\a[22] ), .b(\b[21] ), .c(new_n242), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n251), .b(new_n231), .c(new_n248), .o1(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n250), .c(new_n198), .d(new_n194), .o1(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(new_n257));
  norp02aa1n02x5               g162(.a(\b[23] ), .b(\a[24] ), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(\b[23] ), .b(\a[24] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  aoi112aa1n02x5               g165(.a(new_n255), .b(new_n260), .c(new_n253), .d(new_n257), .o1(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n255), .clkout(new_n262));
  160nm_ficinv00aa1n08x5       g167(.clk(new_n252), .clkout(new_n263));
  aoai13aa1n02x5               g168(.a(new_n257), .b(new_n263), .c(new_n195), .d(new_n249), .o1(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n260), .clkout(new_n265));
  aoi012aa1n02x5               g170(.a(new_n265), .b(new_n264), .c(new_n262), .o1(new_n266));
  norp02aa1n02x5               g171(.a(new_n266), .b(new_n261), .o1(\s[24] ));
  nano23aa1n02x4               g172(.a(new_n255), .b(new_n258), .c(new_n259), .d(new_n256), .out0(new_n268));
  nanp02aa1n02x5               g173(.a(new_n268), .b(new_n248), .o1(new_n269));
  nano22aa1n02x4               g174(.a(new_n269), .b(new_n207), .c(new_n227), .out0(new_n270));
  160nm_ficinv00aa1n08x5       g175(.clk(new_n270), .clkout(new_n271));
  nanp02aa1n02x5               g176(.a(new_n227), .b(new_n223), .o1(new_n272));
  oaoi03aa1n02x5               g177(.a(\a[24] ), .b(\b[23] ), .c(new_n262), .o1(new_n273));
  aoi012aa1n02x5               g178(.a(new_n273), .b(new_n268), .c(new_n251), .o1(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n269), .c(new_n272), .d(new_n230), .o1(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n275), .clkout(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n271), .c(new_n198), .d(new_n194), .o1(new_n277));
  xorb03aa1n02x5               g182(.a(new_n277), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g183(.a(\b[24] ), .b(\a[25] ), .o1(new_n279));
  xorc02aa1n02x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  xorc02aa1n02x5               g185(.a(\a[26] ), .b(\b[25] ), .out0(new_n281));
  aoi112aa1n02x5               g186(.a(new_n279), .b(new_n281), .c(new_n277), .d(new_n280), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n279), .clkout(new_n283));
  aoai13aa1n02x5               g188(.a(new_n280), .b(new_n275), .c(new_n195), .d(new_n270), .o1(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n281), .clkout(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(new_n284), .c(new_n283), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n286), .b(new_n282), .o1(\s[26] ));
  nona23aa1n02x4               g192(.a(new_n239), .b(new_n236), .c(new_n235), .d(new_n238), .out0(new_n288));
  nano22aa1n02x4               g193(.a(new_n288), .b(new_n257), .c(new_n260), .out0(new_n289));
  and002aa1n02x5               g194(.a(new_n281), .b(new_n280), .o(new_n290));
  nano22aa1n02x4               g195(.a(new_n228), .b(new_n290), .c(new_n289), .out0(new_n291));
  160nm_ficinv00aa1n08x5       g196(.clk(new_n291), .clkout(new_n292));
  oao003aa1n02x5               g197(.a(\a[26] ), .b(\b[25] ), .c(new_n283), .carry(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n293), .clkout(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n275), .c(new_n290), .o1(new_n295));
  aoai13aa1n02x5               g200(.a(new_n295), .b(new_n292), .c(new_n198), .d(new_n194), .o1(new_n296));
  xorb03aa1n02x5               g201(.a(new_n296), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  160nm_ficinv00aa1n08x5       g203(.clk(new_n298), .clkout(new_n299));
  xorc02aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .out0(new_n300));
  160nm_ficinv00aa1n08x5       g205(.clk(new_n300), .clkout(new_n301));
  nanp02aa1n02x5               g206(.a(new_n231), .b(new_n289), .o1(new_n302));
  160nm_ficinv00aa1n08x5       g207(.clk(new_n290), .clkout(new_n303));
  aoai13aa1n02x5               g208(.a(new_n293), .b(new_n303), .c(new_n302), .d(new_n274), .o1(new_n304));
  nanp02aa1n02x5               g209(.a(\b[26] ), .b(\a[27] ), .o1(new_n305));
  aoai13aa1n02x5               g210(.a(new_n305), .b(new_n304), .c(new_n195), .d(new_n291), .o1(new_n306));
  aoi012aa1n02x5               g211(.a(new_n301), .b(new_n306), .c(new_n299), .o1(new_n307));
  aoi112aa1n02x5               g212(.a(new_n298), .b(new_n300), .c(new_n296), .d(new_n305), .o1(new_n308));
  norp02aa1n02x5               g213(.a(new_n307), .b(new_n308), .o1(\s[28] ));
  nano22aa1n02x4               g214(.a(new_n301), .b(new_n299), .c(new_n305), .out0(new_n310));
  aoai13aa1n02x5               g215(.a(new_n310), .b(new_n304), .c(new_n195), .d(new_n291), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .c(new_n299), .carry(new_n312));
  xorc02aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  160nm_ficinv00aa1n08x5       g218(.clk(new_n313), .clkout(new_n314));
  aoi012aa1n02x5               g219(.a(new_n314), .b(new_n311), .c(new_n312), .o1(new_n315));
  160nm_ficinv00aa1n08x5       g220(.clk(new_n312), .clkout(new_n316));
  aoi112aa1n02x5               g221(.a(new_n313), .b(new_n316), .c(new_n296), .d(new_n310), .o1(new_n317));
  norp02aa1n02x5               g222(.a(new_n315), .b(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g223(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1n02x4               g224(.a(new_n314), .b(new_n300), .c(new_n305), .d(new_n299), .out0(new_n320));
  aoai13aa1n02x5               g225(.a(new_n320), .b(new_n304), .c(new_n195), .d(new_n291), .o1(new_n321));
  oao003aa1n02x5               g226(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .carry(new_n322));
  xorc02aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .out0(new_n323));
  160nm_ficinv00aa1n08x5       g228(.clk(new_n323), .clkout(new_n324));
  aoi012aa1n02x5               g229(.a(new_n324), .b(new_n321), .c(new_n322), .o1(new_n325));
  160nm_ficinv00aa1n08x5       g230(.clk(new_n322), .clkout(new_n326));
  aoi112aa1n02x5               g231(.a(new_n323), .b(new_n326), .c(new_n296), .d(new_n320), .o1(new_n327));
  norp02aa1n02x5               g232(.a(new_n325), .b(new_n327), .o1(\s[30] ));
  xnrc02aa1n02x5               g233(.a(\b[30] ), .b(\a[31] ), .out0(new_n329));
  160nm_ficinv00aa1n08x5       g234(.clk(new_n329), .clkout(new_n330));
  and003aa1n02x5               g235(.a(new_n310), .b(new_n323), .c(new_n313), .o(new_n331));
  oao003aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(new_n322), .carry(new_n332));
  160nm_ficinv00aa1n08x5       g237(.clk(new_n332), .clkout(new_n333));
  aoi112aa1n02x5               g238(.a(new_n330), .b(new_n333), .c(new_n296), .d(new_n331), .o1(new_n334));
  aoai13aa1n02x5               g239(.a(new_n331), .b(new_n304), .c(new_n195), .d(new_n291), .o1(new_n335));
  aoi012aa1n02x5               g240(.a(new_n329), .b(new_n335), .c(new_n332), .o1(new_n336));
  norp02aa1n02x5               g241(.a(new_n336), .b(new_n334), .o1(\s[31] ));
  xobna2aa1n03x5               g242(.a(new_n104), .b(new_n109), .c(new_n107), .out0(\s[3] ));
  aoai13aa1n02x5               g243(.a(new_n102), .b(new_n104), .c(new_n109), .d(new_n107), .o1(new_n339));
  xorb03aa1n02x5               g244(.a(new_n339), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xobna2aa1n03x5               g245(.a(new_n164), .b(new_n120), .c(new_n119), .out0(\s[5] ));
  oaib12aa1n02x5               g246(.a(new_n120), .b(new_n164), .c(new_n119), .out0(new_n342));
  xnrb03aa1n02x5               g247(.a(new_n342), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aob012aa1n02x5               g248(.a(new_n124), .b(new_n164), .c(new_n121), .out0(new_n344));
  xorb03aa1n02x5               g249(.a(new_n344), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g250(.a(new_n115), .b(new_n344), .c(new_n116), .o1(new_n346));
  xnrb03aa1n02x5               g251(.a(new_n346), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g252(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


