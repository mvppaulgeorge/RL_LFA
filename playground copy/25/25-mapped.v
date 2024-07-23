// Benchmark "adder" written by ABC on Thu Jul 11 12:45:28 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n246,
    new_n247, new_n248, new_n249, new_n250, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n318, new_n319, new_n321,
    new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
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
  orn002aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .o(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n120));
  oaib12aa1n02x5               g025(.a(new_n120), .b(new_n114), .c(new_n119), .out0(new_n121));
  xorc02aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  aoai13aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n123));
  norp02aa1n02x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n123), .c(new_n98), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g032(.clk(new_n126), .clkout(new_n128));
  aoi012aa1n02x5               g033(.a(new_n124), .b(new_n97), .c(new_n125), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n128), .c(new_n123), .d(new_n98), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  norp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoi112aa1n02x5               g042(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n134), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n133), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(\s[12] ));
  nanp02aa1n02x5               g045(.a(new_n109), .b(new_n117), .o1(new_n141));
  nano23aa1n02x4               g046(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n142));
  aobi12aa1n02x5               g047(.a(new_n120), .b(new_n142), .c(new_n119), .out0(new_n143));
  nano23aa1n02x4               g048(.a(new_n132), .b(new_n135), .c(new_n136), .d(new_n133), .out0(new_n144));
  nanp03aa1n02x5               g049(.a(new_n144), .b(new_n122), .c(new_n126), .o1(new_n145));
  nanb03aa1n02x5               g050(.a(new_n129), .b(new_n137), .c(new_n134), .out0(new_n146));
  oai012aa1n02x5               g051(.a(new_n136), .b(new_n135), .c(new_n132), .o1(new_n147));
  and002aa1n02x5               g052(.a(new_n146), .b(new_n147), .o(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n145), .c(new_n141), .d(new_n143), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g055(.clk(\a[14] ), .clkout(new_n151));
  norp02aa1n02x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  xorc02aa1n02x5               g057(.a(\a[13] ), .b(\b[12] ), .out0(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n149), .c(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(new_n151), .out0(\s[14] ));
  norp02aa1n02x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  xorc02aa1n02x5               g063(.a(\a[14] ), .b(\b[13] ), .out0(new_n159));
  and002aa1n02x5               g064(.a(new_n159), .b(new_n153), .o(new_n160));
  160nm_ficinv00aa1n08x5       g065(.clk(\b[13] ), .clkout(new_n161));
  oaoi03aa1n02x5               g066(.a(new_n151), .b(new_n161), .c(new_n152), .o1(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n162), .clkout(new_n163));
  aoai13aa1n02x5               g068(.a(new_n158), .b(new_n163), .c(new_n149), .d(new_n160), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n158), .b(new_n163), .c(new_n149), .d(new_n160), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(\s[15] ));
  norp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  nona22aa1n02x4               g074(.a(new_n164), .b(new_n169), .c(new_n156), .out0(new_n170));
  160nm_ficinv00aa1n08x5       g075(.clk(new_n169), .clkout(new_n171));
  oaoi13aa1n02x5               g076(.a(new_n171), .b(new_n164), .c(\a[15] ), .d(\b[14] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n170), .b(new_n172), .out0(\s[16] ));
  nano23aa1n02x4               g078(.a(new_n156), .b(new_n167), .c(new_n168), .d(new_n157), .out0(new_n174));
  nanp03aa1n02x5               g079(.a(new_n174), .b(new_n153), .c(new_n159), .o1(new_n175));
  norp02aa1n02x5               g080(.a(new_n175), .b(new_n145), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n177));
  aoi012aa1n02x5               g082(.a(new_n175), .b(new_n146), .c(new_n147), .o1(new_n178));
  oai022aa1n02x5               g083(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n179));
  aboi22aa1n03x5               g084(.a(new_n162), .b(new_n174), .c(new_n179), .d(new_n168), .out0(new_n180));
  norb02aa1n02x5               g085(.a(new_n180), .b(new_n178), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n177), .b(new_n181), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[18] ), .clkout(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[17] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\b[16] ), .clkout(new_n186));
  oaoi03aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  xroi22aa1d04x5               g093(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n189));
  norp02aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n191));
  norp02aa1n02x5               g096(.a(new_n191), .b(new_n190), .o1(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n192), .clkout(new_n193));
  norp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n193), .c(new_n182), .d(new_n189), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n182), .d(new_n189), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nona22aa1n02x4               g108(.a(new_n197), .b(new_n203), .c(new_n194), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n194), .clkout(new_n205));
  aobi12aa1n02x5               g110(.a(new_n203), .b(new_n197), .c(new_n205), .out0(new_n206));
  norb02aa1n02x5               g111(.a(new_n204), .b(new_n206), .out0(\s[20] ));
  nona23aa1n02x4               g112(.a(new_n202), .b(new_n195), .c(new_n194), .d(new_n201), .out0(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n208), .clkout(new_n209));
  nanp02aa1n02x5               g114(.a(new_n189), .b(new_n209), .o1(new_n210));
  oai012aa1n02x5               g115(.a(new_n202), .b(new_n201), .c(new_n194), .o1(new_n211));
  oai012aa1n02x5               g116(.a(new_n211), .b(new_n208), .c(new_n192), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n210), .c(new_n177), .d(new_n181), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  aoi112aa1n02x5               g123(.a(new_n216), .b(new_n218), .c(new_n214), .d(new_n217), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n218), .b(new_n216), .c(new_n214), .d(new_n217), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(\s[22] ));
  nanp02aa1n02x5               g126(.a(new_n218), .b(new_n217), .o1(new_n222));
  nanb03aa1n02x5               g127(.a(new_n222), .b(new_n189), .c(new_n209), .out0(new_n223));
  oai112aa1n02x5               g128(.a(new_n196), .b(new_n203), .c(new_n191), .d(new_n190), .o1(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(\a[22] ), .clkout(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(\b[21] ), .clkout(new_n226));
  oaoi03aa1n02x5               g131(.a(new_n225), .b(new_n226), .c(new_n216), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n227), .b(new_n222), .c(new_n224), .d(new_n211), .o1(new_n228));
  160nm_ficinv00aa1n08x5       g133(.clk(new_n228), .clkout(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n223), .c(new_n177), .d(new_n181), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  and002aa1n02x5               g142(.a(new_n234), .b(new_n233), .o(new_n238));
  nona23aa1n02x4               g143(.a(new_n238), .b(new_n189), .c(new_n222), .d(new_n208), .out0(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(\a[24] ), .clkout(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(\b[23] ), .clkout(new_n241));
  oao003aa1n02x5               g146(.a(new_n240), .b(new_n241), .c(new_n232), .carry(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n228), .c(new_n238), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n239), .c(new_n177), .d(new_n181), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g150(.a(\b[24] ), .b(\a[25] ), .o1(new_n246));
  xorc02aa1n02x5               g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  xorc02aa1n02x5               g152(.a(\a[26] ), .b(\b[25] ), .out0(new_n248));
  aoi112aa1n02x5               g153(.a(new_n246), .b(new_n248), .c(new_n244), .d(new_n247), .o1(new_n249));
  aoai13aa1n02x5               g154(.a(new_n248), .b(new_n246), .c(new_n244), .d(new_n247), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n249), .out0(\s[26] ));
  oao003aa1n02x5               g156(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n252));
  nano23aa1n02x4               g157(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n253));
  aobi12aa1n02x5               g158(.a(new_n108), .b(new_n253), .c(new_n252), .out0(new_n254));
  nona22aa1n02x4               g159(.a(new_n142), .b(new_n115), .c(new_n116), .out0(new_n255));
  oai012aa1n02x5               g160(.a(new_n143), .b(new_n254), .c(new_n255), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n180), .b(new_n175), .c(new_n146), .d(new_n147), .o1(new_n257));
  and002aa1n02x5               g162(.a(new_n248), .b(new_n247), .o(new_n258));
  nano22aa1n02x4               g163(.a(new_n223), .b(new_n238), .c(new_n258), .out0(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n257), .c(new_n256), .d(new_n176), .o1(new_n260));
  aoai13aa1n02x5               g165(.a(new_n258), .b(new_n242), .c(new_n228), .d(new_n238), .o1(new_n261));
  oai022aa1n02x5               g166(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n262));
  aob012aa1n02x5               g167(.a(new_n262), .b(\b[25] ), .c(\a[26] ), .out0(new_n263));
  xorc02aa1n02x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  160nm_ficinv00aa1n08x5       g169(.clk(new_n264), .clkout(new_n265));
  aoi013aa1n02x4               g170(.a(new_n265), .b(new_n260), .c(new_n261), .d(new_n263), .o1(new_n266));
  160nm_ficinv00aa1n08x5       g171(.clk(new_n259), .clkout(new_n267));
  aoi012aa1n02x5               g172(.a(new_n267), .b(new_n177), .c(new_n181), .o1(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n222), .clkout(new_n269));
  160nm_ficinv00aa1n08x5       g174(.clk(new_n227), .clkout(new_n270));
  aoai13aa1n02x5               g175(.a(new_n238), .b(new_n270), .c(new_n212), .d(new_n269), .o1(new_n271));
  160nm_ficinv00aa1n08x5       g176(.clk(new_n242), .clkout(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n258), .clkout(new_n273));
  aoai13aa1n02x5               g178(.a(new_n263), .b(new_n273), .c(new_n271), .d(new_n272), .o1(new_n274));
  norp03aa1n02x5               g179(.a(new_n274), .b(new_n268), .c(new_n264), .o1(new_n275));
  norp02aa1n02x5               g180(.a(new_n266), .b(new_n275), .o1(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n277), .clkout(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  nano22aa1n02x4               g184(.a(new_n266), .b(new_n278), .c(new_n279), .out0(new_n280));
  oai012aa1n02x5               g185(.a(new_n264), .b(new_n274), .c(new_n268), .o1(new_n281));
  aoi012aa1n02x5               g186(.a(new_n279), .b(new_n281), .c(new_n278), .o1(new_n282));
  norp02aa1n02x5               g187(.a(new_n282), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g188(.a(new_n264), .b(new_n279), .out0(new_n284));
  oai012aa1n02x5               g189(.a(new_n284), .b(new_n274), .c(new_n268), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[28] ), .b(\a[29] ), .out0(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n285), .c(new_n286), .o1(new_n288));
  160nm_ficinv00aa1n08x5       g193(.clk(new_n284), .clkout(new_n289));
  aoi013aa1n02x4               g194(.a(new_n289), .b(new_n260), .c(new_n261), .d(new_n263), .o1(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n286), .c(new_n287), .out0(new_n291));
  norp02aa1n02x5               g196(.a(new_n288), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g198(.a(new_n264), .b(new_n287), .c(new_n279), .out0(new_n294));
  oai012aa1n02x5               g199(.a(new_n294), .b(new_n274), .c(new_n268), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  160nm_ficinv00aa1n08x5       g203(.clk(new_n294), .clkout(new_n299));
  aoi013aa1n02x4               g204(.a(new_n299), .b(new_n260), .c(new_n261), .d(new_n263), .o1(new_n300));
  nano22aa1n02x4               g205(.a(new_n300), .b(new_n296), .c(new_n297), .out0(new_n301));
  norp02aa1n02x5               g206(.a(new_n298), .b(new_n301), .o1(\s[30] ));
  norb02aa1n02x5               g207(.a(new_n294), .b(new_n297), .out0(new_n303));
  160nm_ficinv00aa1n08x5       g208(.clk(new_n303), .clkout(new_n304));
  aoi013aa1n02x4               g209(.a(new_n304), .b(new_n260), .c(new_n261), .d(new_n263), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nano22aa1n02x4               g212(.a(new_n305), .b(new_n306), .c(new_n307), .out0(new_n308));
  oai012aa1n02x5               g213(.a(new_n303), .b(new_n274), .c(new_n268), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  norp02aa1n02x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g220(.a(\a[5] ), .b(\b[4] ), .c(new_n254), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g222(.a(\a[5] ), .b(\b[4] ), .c(new_n254), .carry(new_n318));
  oaoi03aa1n02x5               g223(.a(\a[6] ), .b(\b[5] ), .c(new_n318), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  160nm_ficinv00aa1n08x5       g225(.clk(\a[8] ), .clkout(new_n321));
  aoi012aa1n02x5               g226(.a(new_n112), .b(new_n319), .c(new_n113), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(new_n321), .out0(\s[8] ));
  xnbna2aa1n03x5               g228(.a(new_n122), .b(new_n141), .c(new_n143), .out0(\s[9] ));
endmodule


