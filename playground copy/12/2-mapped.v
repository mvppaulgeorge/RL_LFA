// Benchmark "adder" written by ABC on Thu Jul 11 11:44:10 2024

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
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n330, new_n333,
    new_n335, new_n337;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  orn002aa1n02x5               g004(.a(\a[9] ), .b(\b[8] ), .o(new_n100));
  norp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  norp02aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nona23aa1n02x4               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  norp02aa1n02x5               g015(.a(new_n110), .b(new_n105), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  160nm_ficinv00aa1n08x5       g017(.clk(new_n112), .clkout(new_n113));
  oaoi03aa1n02x5               g018(.a(\a[4] ), .b(\b[3] ), .c(new_n113), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  norb02aa1n02x5               g021(.a(new_n116), .b(new_n112), .out0(new_n117));
  norp02aa1n02x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  aoi022aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n119));
  oai112aa1n02x5               g024(.a(new_n117), .b(new_n115), .c(new_n119), .d(new_n118), .o1(new_n120));
  nanb02aa1n02x5               g025(.a(new_n114), .b(new_n120), .out0(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(\a[8] ), .clkout(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\b[7] ), .clkout(new_n123));
  nanp02aa1n02x5               g028(.a(new_n123), .b(new_n122), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(new_n103), .b(new_n102), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n126));
  oai112aa1n02x5               g031(.a(new_n124), .b(new_n125), .c(new_n105), .d(new_n126), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n127), .c(new_n121), .d(new_n111), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n100), .out0(\s[10] ));
  nanp02aa1n02x5               g035(.a(\b[0] ), .b(\a[1] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[1] ), .b(\a[2] ), .o1(new_n132));
  aoi012aa1n02x5               g037(.a(new_n118), .b(new_n131), .c(new_n132), .o1(new_n133));
  nano32aa1n02x4               g038(.a(new_n133), .b(new_n113), .c(new_n115), .d(new_n116), .out0(new_n134));
  oai012aa1n02x5               g039(.a(new_n111), .b(new_n134), .c(new_n114), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(new_n124), .b(new_n102), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n103), .b(new_n104), .out0(new_n137));
  norp03aa1n02x5               g042(.a(new_n126), .b(new_n137), .c(new_n136), .o1(new_n138));
  nano22aa1n02x4               g043(.a(new_n138), .b(new_n124), .c(new_n125), .out0(new_n139));
  norp02aa1n02x5               g044(.a(\b[8] ), .b(\a[9] ), .o1(new_n140));
  nona23aa1n02x4               g045(.a(new_n128), .b(new_n98), .c(new_n97), .d(new_n140), .out0(new_n141));
  aoi012aa1n02x5               g046(.a(new_n97), .b(new_n140), .c(new_n98), .o1(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n141), .c(new_n135), .d(new_n139), .o1(new_n143));
  xorb03aa1n02x5               g048(.a(new_n143), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  aoi012aa1n02x5               g051(.a(new_n145), .b(new_n143), .c(new_n146), .o1(new_n147));
  norp02aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  xnrc02aa1n02x5               g055(.a(new_n147), .b(new_n150), .out0(\s[12] ));
  aoi112aa1n02x5               g056(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n152));
  aoi112aa1n02x5               g057(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n146), .b(new_n145), .out0(new_n154));
  oai112aa1n02x5               g059(.a(new_n154), .b(new_n150), .c(new_n153), .d(new_n97), .o1(new_n155));
  nona22aa1n02x4               g060(.a(new_n155), .b(new_n152), .c(new_n148), .out0(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(new_n156), .clkout(new_n157));
  norb02aa1n02x5               g062(.a(new_n128), .b(new_n140), .out0(new_n158));
  nano23aa1n02x4               g063(.a(new_n145), .b(new_n148), .c(new_n149), .d(new_n146), .out0(new_n159));
  nanp03aa1n02x5               g064(.a(new_n159), .b(new_n99), .c(new_n158), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n157), .b(new_n160), .c(new_n135), .d(new_n139), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n163), .b(new_n161), .c(new_n164), .o1(new_n165));
  xnrb03aa1n02x5               g070(.a(new_n165), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  160nm_ficinv00aa1n08x5       g074(.clk(new_n169), .clkout(new_n170));
  norp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  aoi012aa1n02x5               g077(.a(new_n171), .b(new_n163), .c(new_n172), .o1(new_n173));
  160nm_ficinv00aa1n08x5       g078(.clk(new_n173), .clkout(new_n174));
  nano23aa1n02x4               g079(.a(new_n163), .b(new_n171), .c(new_n172), .d(new_n164), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n170), .b(new_n174), .c(new_n161), .d(new_n175), .o1(new_n176));
  aoi112aa1n02x5               g081(.a(new_n174), .b(new_n170), .c(new_n161), .d(new_n175), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(\s[15] ));
  norp02aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  oai112aa1n02x5               g086(.a(new_n176), .b(new_n181), .c(\b[14] ), .d(\a[15] ), .o1(new_n182));
  oaoi13aa1n02x5               g087(.a(new_n181), .b(new_n176), .c(\a[15] ), .d(\b[14] ), .o1(new_n183));
  norb02aa1n02x5               g088(.a(new_n182), .b(new_n183), .out0(\s[16] ));
  nano23aa1n02x4               g089(.a(new_n167), .b(new_n179), .c(new_n180), .d(new_n168), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n185), .b(new_n175), .o1(new_n186));
  nano32aa1n02x4               g091(.a(new_n186), .b(new_n159), .c(new_n158), .d(new_n99), .out0(new_n187));
  aoai13aa1n02x5               g092(.a(new_n187), .b(new_n127), .c(new_n121), .d(new_n111), .o1(new_n188));
  nona23aa1n02x4               g093(.a(new_n172), .b(new_n164), .c(new_n163), .d(new_n171), .out0(new_n189));
  norp03aa1n02x5               g094(.a(new_n189), .b(new_n181), .c(new_n169), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n191));
  160nm_fiao0012aa1n02p5x5     g096(.a(new_n179), .b(new_n185), .c(new_n174), .o(new_n192));
  aoi112aa1n02x5               g097(.a(new_n192), .b(new_n191), .c(new_n156), .d(new_n190), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(new_n193), .b(new_n188), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nona23aa1n02x4               g100(.a(new_n185), .b(new_n159), .c(new_n141), .d(new_n189), .out0(new_n196));
  aoi012aa1n02x5               g101(.a(new_n196), .b(new_n135), .c(new_n139), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(new_n156), .b(new_n190), .o1(new_n198));
  norp03aa1n02x5               g103(.a(new_n173), .b(new_n181), .c(new_n169), .o1(new_n199));
  nona32aa1n02x4               g104(.a(new_n198), .b(new_n199), .c(new_n191), .d(new_n179), .out0(new_n200));
  norp02aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[16] ), .b(\a[17] ), .o1(new_n202));
  oaoi13aa1n02x5               g107(.a(new_n201), .b(new_n202), .c(new_n200), .d(new_n197), .o1(new_n203));
  xnrb03aa1n02x5               g108(.a(new_n203), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  norp02aa1n02x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nano23aa1n02x4               g111(.a(new_n201), .b(new_n205), .c(new_n206), .d(new_n202), .out0(new_n207));
  160nm_ficinv00aa1n08x5       g112(.clk(new_n207), .clkout(new_n208));
  160nm_fiao0012aa1n02p5x5     g113(.a(new_n205), .b(new_n201), .c(new_n206), .o(new_n209));
  160nm_ficinv00aa1n08x5       g114(.clk(new_n209), .clkout(new_n210));
  aoai13aa1n02x5               g115(.a(new_n210), .b(new_n208), .c(new_n193), .d(new_n188), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  norp02aa1n02x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nanp02aa1n02x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  aoi112aa1n02x5               g123(.a(new_n214), .b(new_n218), .c(new_n211), .d(new_n215), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n218), .b(new_n214), .c(new_n211), .d(new_n215), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(\s[20] ));
  nano23aa1n02x4               g126(.a(new_n214), .b(new_n216), .c(new_n217), .d(new_n215), .out0(new_n222));
  nanp02aa1n02x5               g127(.a(new_n222), .b(new_n207), .o1(new_n223));
  aoi112aa1n02x5               g128(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n224));
  aoi112aa1n02x5               g129(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n215), .b(new_n214), .out0(new_n226));
  oai112aa1n02x5               g131(.a(new_n226), .b(new_n218), .c(new_n225), .d(new_n205), .o1(new_n227));
  nona22aa1n02x4               g132(.a(new_n227), .b(new_n224), .c(new_n216), .out0(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n228), .clkout(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n223), .c(new_n193), .d(new_n188), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  norp02aa1n02x5               g139(.a(\b[21] ), .b(\a[22] ), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(\b[21] ), .b(\a[22] ), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(new_n232), .b(new_n237), .c(new_n230), .d(new_n234), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n237), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(\s[22] ));
  nano23aa1n02x4               g145(.a(new_n232), .b(new_n235), .c(new_n236), .d(new_n233), .out0(new_n241));
  aoi012aa1n02x5               g146(.a(new_n235), .b(new_n232), .c(new_n236), .o1(new_n242));
  aobi12aa1n02x5               g147(.a(new_n242), .b(new_n228), .c(new_n241), .out0(new_n243));
  nanp03aa1n02x5               g148(.a(new_n222), .b(new_n207), .c(new_n241), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n243), .b(new_n244), .c(new_n193), .d(new_n188), .o1(new_n245));
  norp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n248), .clkout(new_n249));
  aoai13aa1n02x5               g154(.a(new_n249), .b(new_n244), .c(new_n193), .d(new_n188), .o1(new_n250));
  aboi22aa1n03x5               g155(.a(new_n250), .b(new_n243), .c(new_n245), .d(new_n248), .out0(\s[23] ));
  norp02aa1n02x5               g156(.a(\b[23] ), .b(\a[24] ), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  nanb02aa1n02x5               g158(.a(new_n252), .b(new_n253), .out0(new_n254));
  160nm_ficinv00aa1n08x5       g159(.clk(new_n254), .clkout(new_n255));
  aoi112aa1n02x5               g160(.a(new_n246), .b(new_n255), .c(new_n245), .d(new_n248), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n255), .b(new_n246), .c(new_n245), .d(new_n247), .o1(new_n257));
  norb02aa1n02x5               g162(.a(new_n257), .b(new_n256), .out0(\s[24] ));
  nona23aa1n02x4               g163(.a(new_n253), .b(new_n247), .c(new_n246), .d(new_n252), .out0(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n259), .clkout(new_n260));
  nanb03aa1n02x5               g165(.a(new_n223), .b(new_n260), .c(new_n241), .out0(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n252), .clkout(new_n262));
  nanp02aa1n02x5               g167(.a(new_n246), .b(new_n253), .o1(new_n263));
  oai112aa1n02x5               g168(.a(new_n263), .b(new_n262), .c(new_n259), .d(new_n242), .o1(new_n264));
  nano22aa1n02x4               g169(.a(new_n259), .b(new_n234), .c(new_n237), .out0(new_n265));
  aoi012aa1n02x5               g170(.a(new_n264), .b(new_n228), .c(new_n265), .o1(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n261), .c(new_n193), .d(new_n188), .o1(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  xorc02aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .out0(new_n271));
  aoi112aa1n02x5               g176(.a(new_n269), .b(new_n271), .c(new_n267), .d(new_n270), .o1(new_n272));
  aoai13aa1n02x5               g177(.a(new_n271), .b(new_n269), .c(new_n267), .d(new_n270), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n273), .b(new_n272), .out0(\s[26] ));
  nano32aa1n02x4               g179(.a(new_n244), .b(new_n271), .c(new_n260), .d(new_n270), .out0(new_n275));
  oai012aa1n02x5               g180(.a(new_n275), .b(new_n200), .c(new_n197), .o1(new_n276));
  norp02aa1n02x5               g181(.a(\b[25] ), .b(\a[26] ), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n277), .clkout(new_n278));
  aoi112aa1n02x5               g183(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n279));
  160nm_ficinv00aa1n08x5       g184(.clk(new_n279), .clkout(new_n280));
  norb03aa1n02x5               g185(.a(new_n248), .b(new_n242), .c(new_n254), .out0(new_n281));
  nano22aa1n02x4               g186(.a(new_n281), .b(new_n262), .c(new_n263), .out0(new_n282));
  nanp02aa1n02x5               g187(.a(new_n228), .b(new_n265), .o1(new_n283));
  and002aa1n02x5               g188(.a(new_n271), .b(new_n270), .o(new_n284));
  aobi12aa1n02x5               g189(.a(new_n284), .b(new_n283), .c(new_n282), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n285), .b(new_n278), .c(new_n280), .out0(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .out0(new_n287));
  xobna2aa1n03x5               g192(.a(new_n287), .b(new_n286), .c(new_n276), .out0(\s[27] ));
  norp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  160nm_ficinv00aa1n08x5       g194(.clk(new_n289), .clkout(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[27] ), .b(\a[28] ), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n284), .b(new_n264), .c(new_n228), .d(new_n265), .o1(new_n292));
  nona22aa1n02x4               g197(.a(new_n292), .b(new_n279), .c(new_n277), .out0(new_n293));
  and002aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o(new_n294));
  160nm_ficinv00aa1n08x5       g199(.clk(new_n294), .clkout(new_n295));
  aoai13aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n194), .d(new_n275), .o1(new_n296));
  aoi012aa1n02x5               g201(.a(new_n291), .b(new_n296), .c(new_n290), .o1(new_n297));
  aoi012aa1n02x5               g202(.a(new_n294), .b(new_n286), .c(new_n276), .o1(new_n298));
  nano22aa1n02x4               g203(.a(new_n298), .b(new_n290), .c(new_n291), .out0(new_n299));
  norp02aa1n02x5               g204(.a(new_n297), .b(new_n299), .o1(\s[28] ));
  norp02aa1n02x5               g205(.a(new_n291), .b(new_n287), .o1(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n293), .c(new_n194), .d(new_n275), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[28] ), .b(\b[27] ), .c(new_n290), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[28] ), .b(\a[29] ), .out0(new_n304));
  aoi012aa1n02x5               g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  160nm_ficinv00aa1n08x5       g210(.clk(new_n301), .clkout(new_n306));
  aoi012aa1n02x5               g211(.a(new_n306), .b(new_n286), .c(new_n276), .o1(new_n307));
  nano22aa1n02x4               g212(.a(new_n307), .b(new_n303), .c(new_n304), .out0(new_n308));
  norp02aa1n02x5               g213(.a(new_n305), .b(new_n308), .o1(\s[29] ));
  xorb03aa1n02x5               g214(.a(new_n131), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norp03aa1n02x5               g215(.a(new_n304), .b(new_n291), .c(new_n287), .o1(new_n311));
  aoai13aa1n02x5               g216(.a(new_n311), .b(new_n293), .c(new_n194), .d(new_n275), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .c(new_n303), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[29] ), .b(\a[30] ), .out0(new_n314));
  aoi012aa1n02x5               g219(.a(new_n314), .b(new_n312), .c(new_n313), .o1(new_n315));
  160nm_ficinv00aa1n08x5       g220(.clk(new_n311), .clkout(new_n316));
  aoi012aa1n02x5               g221(.a(new_n316), .b(new_n286), .c(new_n276), .o1(new_n317));
  nano22aa1n02x4               g222(.a(new_n317), .b(new_n313), .c(new_n314), .out0(new_n318));
  norp02aa1n02x5               g223(.a(new_n315), .b(new_n318), .o1(\s[30] ));
  xnrc02aa1n02x5               g224(.a(\b[30] ), .b(\a[31] ), .out0(new_n320));
  norb03aa1n02x5               g225(.a(new_n301), .b(new_n314), .c(new_n304), .out0(new_n321));
  160nm_ficinv00aa1n08x5       g226(.clk(new_n321), .clkout(new_n322));
  aoi012aa1n02x5               g227(.a(new_n322), .b(new_n286), .c(new_n276), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[30] ), .b(\b[29] ), .c(new_n313), .carry(new_n324));
  nano22aa1n02x4               g229(.a(new_n323), .b(new_n320), .c(new_n324), .out0(new_n325));
  aoai13aa1n02x5               g230(.a(new_n321), .b(new_n293), .c(new_n194), .d(new_n275), .o1(new_n326));
  aoi012aa1n02x5               g231(.a(new_n320), .b(new_n326), .c(new_n324), .o1(new_n327));
  norp02aa1n02x5               g232(.a(new_n327), .b(new_n325), .o1(\s[31] ));
  xnbna2aa1n03x5               g233(.a(new_n133), .b(new_n113), .c(new_n116), .out0(\s[3] ));
  oaoi13aa1n02x5               g234(.a(new_n112), .b(new_n116), .c(new_n119), .d(new_n118), .o1(new_n330));
  xnrb03aa1n02x5               g235(.a(new_n330), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g236(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi13aa1n02x5               g237(.a(new_n108), .b(new_n109), .c(new_n134), .d(new_n114), .o1(new_n333));
  xnrb03aa1n02x5               g238(.a(new_n333), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g239(.a(new_n126), .b(new_n110), .c(new_n121), .out0(new_n335));
  xorb03aa1n02x5               g240(.a(new_n335), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g241(.a(new_n103), .b(new_n335), .c(new_n104), .o1(new_n337));
  xnbna2aa1n03x5               g242(.a(new_n337), .b(new_n124), .c(new_n102), .out0(\s[8] ));
  xnbna2aa1n03x5               g243(.a(new_n158), .b(new_n135), .c(new_n139), .out0(\s[9] ));
endmodule


